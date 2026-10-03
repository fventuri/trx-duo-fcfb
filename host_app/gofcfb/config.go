// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// Minimal INI/TOML-style config parser (hand-written, stdlib only -- Go has no
// stdlib TOML, and staying dependency-free is a requirement).  Supports:
//
//	[board]
//	ip = 192.168.255.20
//	udp_port = 55055        # optional (default 55055)
//	guard = 3               # optional
//	workers = 8             # optional (decoder subprocess pool size)
//	recon_workers = 1       # optional: parallel reconstruction goroutines (default
//	                        # 1 = the single-threaded ingest path).  >1 splits the
//	                        # per-channel synth/resample across that many cores; the
//	                        # ingest goroutine still owns seq accounting.  Only worth
//	                        # it past ~100 channels (see NOTE_GOFARM_PARALLEL_*).
//	mtu = 3980              # optional: if set, the host interface reaching the
//	                        # board must have this exact MTU or the farm fails at
//	                        # startup (jumbo frames are silently dropped otherwise)
//	replay = capture.bin    # optional: decode a v3 .bin instead of the board
//
//	[decoder]               # one per decoder; channels reference these by name
//	name    = ft8
//	cmd     = jt9 --ft8 {wav}   # placeholders: {wav} {fmhz} {fhz} {utc}
//	parser  = jt9              # jt9 | wspr (output format)
//	period  = 15               # UTC window period (s)
//	capture = 13.5             # capture length within the period (s)
//	rate    = 12000            # optional WAV sample rate (default 12000)
//
//	[decoder]
//	name    = wspr
//	cmd     = wsprd -f {fmhz} {wav}
//	parser  = wspr
//	period  = 120
//	capture = 114
//
//	[channel]               # repeat [channel] for each receiver
//	fc = 7.074e6
//	decoder = ft8           # references a [decoder] by name
//	adc = 1                 # 1=A, 2=B, 3=both (diversity: one stereo WAV, L=A R=B)
//	bw = 3000               # optional occupied bandwidth (Hz)
//	name = 40mF8
package fcfb

import (
	"bufio"
	"fmt"
	"os"
	"strconv"
	"strings"
)

type Channel struct {
	FcHz    float64
	Decoder string // name of a [decoder]
	Adc     int
	BwHz    float64
	Name    string
}

func (c Channel) name() string {
	if c.Name != "" {
		return c.Name
	}
	return fmt.Sprintf("%.4f", c.FcHz/1e6)
}

type Config struct {
	BoardIP         string
	UDPPort         int
	Guard           int
	Workers         int
	ReconWorkers    int // parallel reconstruction groups (1 = single-threaded ingest)
	MTU             int
	Replay          string
	Kernel          string // optional: external synthesis kernel (.f64); default = embedded K=6
	MonitorInterval int    // board temperature-log poll interval (s); 0 = off
	Decoders        []Decoder
	Channels        []Channel
}

func defaultConfig() Config {
	return Config{UDPPort: 55055, Guard: guardDef, Workers: 8, ReconWorkers: 1, MTU: 0,
		MonitorInterval: monitorIntervalDef}
}

// decoder looks up a [decoder] by name.
func (c Config) decoder(name string) (Decoder, bool) {
	for _, d := range c.Decoders {
		if d.Name == name {
			return d, true
		}
	}
	return Decoder{}, false
}

func parseConfig(path string) (Config, error) {
	cfg := defaultConfig()
	f, err := os.Open(path)
	if err != nil {
		return cfg, err
	}
	defer f.Close()

	section := ""
	var curCh *Channel  // current [channel] being filled
	var curDec *Decoder // current [decoder] being filled
	sc := bufio.NewScanner(f)
	ln := 0
	for sc.Scan() {
		ln++
		line := strings.TrimSpace(sc.Text())
		if line == "" || strings.HasPrefix(line, "#") || strings.HasPrefix(line, ";") {
			continue
		}
		if strings.HasPrefix(line, "[") && strings.HasSuffix(line, "]") {
			section = strings.ToLower(strings.TrimSpace(line[1 : len(line)-1]))
			// accept [channel] and TOML array-of-tables [[channel]]
			section = strings.Trim(section, "[]")
			switch section {
			case "channel":
				cfg.Channels = append(cfg.Channels, Channel{Adc: 1, BwHz: 3000})
				curCh = &cfg.Channels[len(cfg.Channels)-1]
			case "decoder":
				cfg.Decoders = append(cfg.Decoders, Decoder{})
				curDec = &cfg.Decoders[len(cfg.Decoders)-1]
			}
			continue
		}
		key, val, ok := splitKV(line)
		if !ok {
			return cfg, fmt.Errorf("line %d: expected key = value, got %q", ln, line)
		}
		switch section {
		case "board", "":
			if err := setBoard(&cfg, key, val); err != nil {
				return cfg, fmt.Errorf("line %d: %w", ln, err)
			}
		case "channel":
			if curCh == nil {
				return cfg, fmt.Errorf("line %d: key %q before any [channel]", ln, key)
			}
			if err := setChannel(curCh, key, val); err != nil {
				return cfg, fmt.Errorf("line %d: %w", ln, err)
			}
		case "decoder":
			if curDec == nil {
				return cfg, fmt.Errorf("line %d: key %q before any [decoder]", ln, key)
			}
			if err := setDecoder(curDec, key, val); err != nil {
				return cfg, fmt.Errorf("line %d: %w", ln, err)
			}
		default:
			return cfg, fmt.Errorf("line %d: unknown section [%s]", ln, section)
		}
	}
	if err := sc.Err(); err != nil {
		return cfg, err
	}
	if cfg.Replay == "" && cfg.BoardIP == "" {
		return cfg, fmt.Errorf("config: [board] ip is required (or set replay =)")
	}
	if err := validateDecoders(cfg.Decoders); err != nil {
		return cfg, err
	}
	if len(cfg.Channels) == 0 {
		return cfg, fmt.Errorf("config: at least one [channel] is required")
	}
	for i := range cfg.Channels {
		if cfg.Channels[i].Decoder == "" {
			return cfg, fmt.Errorf("channel %d (%s): decoder = is required", i, cfg.Channels[i].name())
		}
		if _, ok := cfg.decoder(cfg.Channels[i].Decoder); !ok {
			return cfg, fmt.Errorf("channel %d (%s): no [decoder] named %q", i, cfg.Channels[i].name(), cfg.Channels[i].Decoder)
		}
	}
	return cfg, nil
}

// validateDecoders checks each [decoder] is complete and names are unique.
func validateDecoders(decs []Decoder) error {
	if len(decs) == 0 {
		return fmt.Errorf("config: at least one [decoder] is required")
	}
	seen := map[string]bool{}
	for i, d := range decs {
		if d.Name == "" {
			return fmt.Errorf("decoder %d: name = is required", i)
		}
		if seen[d.Name] {
			return fmt.Errorf("decoder %q: duplicate name", d.Name)
		}
		seen[d.Name] = true
		if strings.TrimSpace(d.Cmd) == "" {
			return fmt.Errorf("decoder %q: cmd = is required", d.Name)
		}
		if d.Parser != "jt9" && d.Parser != "fst4w" && d.Parser != "wspr" {
			return fmt.Errorf("decoder %q: parser must be jt9, fst4w or wspr (got %q)", d.Name, d.Parser)
		}
		if d.PeriodS <= 0 || d.CaptureS <= 0 {
			return fmt.Errorf("decoder %q: period and capture must be > 0", d.Name)
		}
		if d.CaptureS > d.PeriodS {
			return fmt.Errorf("decoder %q: capture (%.3f) exceeds period (%.3f)", d.Name, d.CaptureS, d.PeriodS)
		}
		if d.Rate < 0 {
			return fmt.Errorf("decoder %q: rate must be > 0 (omit for the 12000 default)", d.Name)
		}
	}
	return nil
}

func splitKV(line string) (string, string, bool) {
	// strip inline comments (a # or ; not inside a value we care about)
	if i := strings.IndexAny(line, "#;"); i >= 0 {
		line = strings.TrimSpace(line[:i])
	}
	i := strings.Index(line, "=")
	if i < 0 {
		return "", "", false
	}
	k := strings.TrimSpace(line[:i])
	v := strings.TrimSpace(strings.Trim(strings.TrimSpace(line[i+1:]), `"'`))
	return strings.ToLower(k), v, k != ""
}

func setBoard(c *Config, k, v string) error {
	var err error
	switch k {
	case "ip":
		c.BoardIP = v
	case "udp_port":
		c.UDPPort, err = strconv.Atoi(v)
	case "guard":
		c.Guard, err = strconv.Atoi(v)
	case "workers":
		c.Workers, err = strconv.Atoi(v)
	case "recon_workers":
		c.ReconWorkers, err = strconv.Atoi(v)
		if err == nil && c.ReconWorkers < 1 {
			return fmt.Errorf("recon_workers must be >= 1, got %d", c.ReconWorkers)
		}
	case "mtu":
		c.MTU, err = strconv.Atoi(v)
	case "replay":
		c.Replay = v
	case "kernel":
		c.Kernel = v
	case "monitor_interval":
		c.MonitorInterval, err = strconv.Atoi(v)
		if err == nil && c.MonitorInterval < 0 {
			return fmt.Errorf("monitor_interval must be >= 0, got %d", c.MonitorInterval)
		}
	default:
		return fmt.Errorf("unknown [board] key %q", k)
	}
	return err
}

func setChannel(c *Channel, k, v string) error {
	var err error
	switch k {
	case "fc":
		c.FcHz, err = strconv.ParseFloat(v, 64)
	case "decoder":
		c.Decoder = v
	case "adc":
		c.Adc, err = strconv.Atoi(v)
		if err == nil && (c.Adc < 1 || c.Adc > 3) {
			return fmt.Errorf("adc must be 1 (A), 2 (B) or 3 (both/diversity), got %d", c.Adc)
		}
	case "bw":
		c.BwHz, err = strconv.ParseFloat(v, 64)
	case "name":
		c.Name = v
	default:
		return fmt.Errorf("unknown [channel] key %q", k)
	}
	return err
}

func setDecoder(d *Decoder, k, v string) error {
	var err error
	switch k {
	case "name":
		d.Name = v
	case "cmd":
		d.Cmd = v
	case "parser":
		d.Parser = strings.ToLower(v)
	case "period":
		d.PeriodS, err = strconv.ParseFloat(v, 64)
	case "capture":
		d.CaptureS, err = strconv.ParseFloat(v, 64)
	case "rate":
		d.Rate, err = strconv.Atoi(v)
	default:
		return fmt.Errorf("unknown [decoder] key %q", k)
	}
	return err
}
