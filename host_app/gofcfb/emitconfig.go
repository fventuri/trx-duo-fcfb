// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// INI/TOML-style config for fcfbhpsdr, same shape as fcfbfarm's config.go (shared
// splitKV parser).  Unlike the farm, the emitter's DDCs/frequencies/rates come from
// the P2 CLIENT at runtime, so the config only describes the board connection and the
// radio identities:
//
//	[board]
//	ip        = 192.168.255.20
//	udp_port  = 55055        # optional (default 55055): local UDP data port
//	guard     = 3            # optional: guard bins each side (non-default needs a v4 server)
//	mtu       = 3980         # optional (default 1500): MUST match the server's -u; sets the budget
//	budget    = 655          # optional: override the bin budget (union W_a+W_b cap) directly
//	gain      = 1.0          # optional: I/Q output gain
//	bin_width = 24           # optional (default 24): wire bin width, for the budget formula
//	kernel    = dual_K4.f64  # optional external synthesis kernel (default = embedded K=6)
//	recon_workers = 4        # optional (default 1): parallel per-DDC reconstruction
//	                         # goroutines; >1 spreads synth+resample across cores
//
//	[radio]                  # repeat [radio] per radio identity; omit for one on 0.0.0.0
//	bind      = 192.168.255.2
//	mac       = 02:00:00:fc:fb:02   # optional (default: locally-administered, per index)
//	board_id  = angelia             # optional: angelia|hermes|... or a number (default angelia)
//	n_ddc     = 8                   # optional (default 8): DDCs advertised
package fcfb

import (
	"bufio"
	"fmt"
	"os"
	"strconv"
	"strings"
)

// HPSDRConfig is the parsed fcfbhpsdr configuration.
type HPSDRConfig struct {
	BoardIP         string
	UDPPort         int
	Guard           int
	MTU             int
	Budget          int
	Gain            float64
	BinWidth        int
	Kernel          string
	ReconWorkers    int
	MonitorInterval int // board temperature-log poll interval (s); 0 = off
	Radios          []RadioSpec
}

func defaultHPSDRConfig() HPSDRConfig {
	return HPSDRConfig{UDPPort: 55055, Guard: guardDef, MTU: 1500, Gain: 1.0, BinWidth: 24,
		ReconWorkers: 1, MonitorInterval: monitorIntervalDef}
}

// boardIDByName maps an advertised HPSDR model name to its board id (byte 11).
func boardIDByName(s string) (int, bool) {
	switch strings.ToLower(strings.TrimSpace(s)) {
	case "atlas":
		return boardAtlas, true
	case "hermes":
		return boardHermes, true
	case "hermes2":
		return boardHermes2, true
	case "angelia":
		return boardAngelia, true
	case "orion":
		return boardOrion, true
	case "orion2":
		return boardOrion2, true
	}
	return 0, false
}

// parseMAC parses "aa:bb:cc:dd:ee:ff" (or '-' separated) into 6 bytes.
func parseMAC(s string) ([6]byte, error) {
	var mac [6]byte
	parts := strings.FieldsFunc(s, func(r rune) bool { return r == ':' || r == '-' })
	if len(parts) != 6 {
		return mac, fmt.Errorf("mac %q: want 6 hex octets", s)
	}
	for i, p := range parts {
		v, err := strconv.ParseUint(strings.TrimSpace(p), 16, 8)
		if err != nil {
			return mac, fmt.Errorf("mac %q: bad octet %q", s, p)
		}
		mac[i] = byte(v)
	}
	return mac, nil
}

func parseHPSDRConfig(path string) (HPSDRConfig, error) {
	cfg := defaultHPSDRConfig()
	f, err := os.Open(path)
	if err != nil {
		return cfg, err
	}
	defer f.Close()

	section := ""
	var curRadio *RadioSpec
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
			section = strings.Trim(section, "[]") // accept [[radio]] too
			if section == "radio" {
				cfg.Radios = append(cfg.Radios, RadioSpec{})
				curRadio = &cfg.Radios[len(cfg.Radios)-1]
			}
			continue
		}
		key, val, ok := splitKV(line)
		if !ok {
			return cfg, fmt.Errorf("line %d: expected key = value, got %q", ln, line)
		}
		switch section {
		case "board", "":
			if err := setEmitBoard(&cfg, key, val); err != nil {
				return cfg, fmt.Errorf("line %d: %w", ln, err)
			}
		case "radio":
			if curRadio == nil {
				return cfg, fmt.Errorf("line %d: key %q before any [radio]", ln, key)
			}
			if err := setRadio(curRadio, key, val); err != nil {
				return cfg, fmt.Errorf("line %d: %w", ln, err)
			}
		default:
			return cfg, fmt.Errorf("line %d: unknown section [%s]", ln, section)
		}
	}
	if err := sc.Err(); err != nil {
		return cfg, err
	}
	if cfg.BoardIP == "" {
		return cfg, fmt.Errorf("config: [board] ip is required")
	}
	return cfg, nil
}

func setEmitBoard(c *HPSDRConfig, k, v string) error {
	var err error
	switch k {
	case "ip":
		c.BoardIP = v
	case "udp_port":
		c.UDPPort, err = strconv.Atoi(v)
	case "guard":
		c.Guard, err = strconv.Atoi(v)
	case "mtu":
		c.MTU, err = strconv.Atoi(v)
	case "budget":
		c.Budget, err = strconv.Atoi(v)
	case "gain":
		c.Gain, err = strconv.ParseFloat(v, 64)
	case "bin_width":
		c.BinWidth, err = strconv.Atoi(v)
	case "kernel":
		c.Kernel = v
	case "recon_workers":
		c.ReconWorkers, err = strconv.Atoi(v)
		if err == nil && c.ReconWorkers < 1 {
			return fmt.Errorf("recon_workers must be >= 1, got %d", c.ReconWorkers)
		}
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

func setRadio(r *RadioSpec, k, v string) error {
	switch k {
	case "bind":
		r.BindIP = v
	case "mac":
		mac, err := parseMAC(v)
		if err != nil {
			return err
		}
		r.MAC = &mac
	case "board_id":
		if id, err := strconv.Atoi(v); err == nil {
			r.BoardID = id
		} else if id, ok := boardIDByName(v); ok {
			r.BoardID = id
		} else {
			return fmt.Errorf("board_id %q: not a number or known model (angelia|hermes|...)", v)
		}
	case "n_ddc":
		n, err := strconv.Atoi(v)
		if err != nil {
			return err
		}
		r.NDDC = n
	default:
		return fmt.Errorf("unknown [radio] key %q", k)
	}
	return nil
}
