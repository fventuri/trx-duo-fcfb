// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
package fcfb

import (
	"os"
	"path/filepath"
	"testing"
)

func writeTemp(t *testing.T, body string) string {
	t.Helper()
	p := filepath.Join(t.TempDir(), "hpsdr.ini")
	if err := os.WriteFile(p, []byte(body), 0o644); err != nil {
		t.Fatal(err)
	}
	return p
}

func TestParseHPSDRConfigFull(t *testing.T) {
	cfg, err := parseHPSDRConfig(writeTemp(t, `
[board]
ip        = 192.168.255.20
udp_port  = 55055
guard     = 5
mtu       = 3980
budget    = 600
gain      = 2.5
bin_width = 24
recon_workers = 4

[radio]
bind     = 192.168.255.2
mac      = 02:00:00:fc:fb:02
board_id = angelia
n_ddc    = 8

[radio]
bind     = 192.168.255.3
board_id = 1
n_ddc    = 4
`))
	if err != nil {
		t.Fatal(err)
	}
	if cfg.BoardIP != "192.168.255.20" || cfg.UDPPort != 55055 || cfg.Guard != 5 ||
		cfg.MTU != 3980 || cfg.Budget != 600 || cfg.Gain != 2.5 || cfg.BinWidth != 24 ||
		cfg.ReconWorkers != 4 {
		t.Fatalf("board fields: %+v", cfg)
	}
	if len(cfg.Radios) != 2 {
		t.Fatalf("radios: %d", len(cfg.Radios))
	}
	r0 := cfg.Radios[0]
	if r0.BindIP != "192.168.255.2" || r0.BoardID != boardAngelia || r0.NDDC != 8 ||
		r0.MAC == nil || *r0.MAC != ([6]byte{0x02, 0, 0, 0xFC, 0xFB, 0x02}) {
		t.Fatalf("radio0: %+v mac=%v", r0, r0.MAC)
	}
	r1 := cfg.Radios[1]
	if r1.BindIP != "192.168.255.3" || r1.BoardID != boardHermes || r1.NDDC != 4 || r1.MAC != nil {
		t.Fatalf("radio1: %+v", r1)
	}
}

func TestParseHPSDRConfigDefaultsAndErrors(t *testing.T) {
	// board-only: defaults applied, no radios (NewEmitter then makes one on the flag bind).
	cfg, err := parseHPSDRConfig(writeTemp(t, "[board]\nip = 10.0.0.5\n"))
	if err != nil {
		t.Fatal(err)
	}
	if cfg.UDPPort != 55055 || cfg.Guard != guardDef || cfg.MTU != 1500 || cfg.Gain != 1.0 ||
		cfg.BinWidth != 24 || cfg.ReconWorkers != 1 {
		t.Fatalf("defaults: %+v", cfg)
	}
	// recon_workers < 1 is rejected.
	if _, err := parseHPSDRConfig(writeTemp(t, "[board]\nip = x\nrecon_workers = 0\n")); err == nil {
		t.Fatal("expected error for recon_workers = 0")
	}
	if len(cfg.Radios) != 0 {
		t.Fatalf("expected no radios, got %d", len(cfg.Radios))
	}
	// missing ip -> error
	if _, err := parseHPSDRConfig(writeTemp(t, "[board]\nguard = 3\n")); err == nil {
		t.Fatal("expected error for missing ip")
	}
	// unknown key -> error
	if _, err := parseHPSDRConfig(writeTemp(t, "[board]\nip = x\nbogus = 1\n")); err == nil {
		t.Fatal("expected error for unknown key")
	}
	// bad board_id -> error
	if _, err := parseHPSDRConfig(writeTemp(t, "[board]\nip = x\n[radio]\nboard_id = wombat\n")); err == nil {
		t.Fatal("expected error for unknown board_id")
	}
}

func TestParseMAC(t *testing.T) {
	m, err := parseMAC("aa:BB:cc:dd:ee:0f")
	if err != nil || m != [6]byte{0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x0F} {
		t.Fatalf("mac colon: %v %v", m, err)
	}
	if _, err := parseMAC("aa-bb-cc-dd-ee-ff"); err != nil {
		t.Fatalf("mac dash: %v", err)
	}
	if _, err := parseMAC("aa:bb:cc"); err == nil {
		t.Fatal("expected error for short mac")
	}
}
