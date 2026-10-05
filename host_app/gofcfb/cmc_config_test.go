// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
package fcfb

import (
	"os"
	"path/filepath"
	"strings"
	"testing"
)

// TestCMCRequiresIQ: a decoder whose cmd invokes common-mode-cancelling must fail
// config parsing unless [fcfbfarm] wav_format = iq is set (the wrapper needs the
// 4-channel I/Q WAV; without iq every decode would silently error out).
func TestCMCRequiresIQ(t *testing.T) {
	write := func(body string) string {
		p := filepath.Join(t.TempDir(), "farm.ini")
		if err := os.WriteFile(p, []byte(body), 0o644); err != nil {
			t.Fatal(err)
		}
		return p
	}
	cmcDecoder := "[decoder]\nname=ft8\ncmd=common-mode-cancelling jt9 --ft8 {wav}\n" +
		"parser=jt9\nperiod=15\ncapture=13.5\n"
	chCfg := "[channel]\nfc=7.074e6\ndecoder=ft8\nadc=3\nname=40mF8\n"
	board := "[board]\nreplay=x.bin\n"

	// No wav_format (default audio) -> must fail, naming the fix.
	_, err := parseConfig(write(board + cmcDecoder + chCfg))
	if err == nil {
		t.Fatal("common-mode-cancelling without wav_format=iq should fail")
	}
	if !strings.Contains(err.Error(), "wav_format = iq") {
		t.Fatalf("error should point at wav_format = iq, got: %v", err)
	}

	// Explicit wav_format = audio -> still must fail.
	if _, err := parseConfig(write(board + "[fcfbfarm]\nwav_format=audio\n" +
		cmcDecoder + chCfg)); err == nil {
		t.Fatal("common-mode-cancelling with wav_format=audio should fail")
	}

	// wav_format = iq -> accepted.
	if _, err := parseConfig(write(board + "[fcfbfarm]\nwav_format=iq\n" +
		cmcDecoder + chCfg)); err != nil {
		t.Fatalf("common-mode-cancelling with wav_format=iq should parse: %v", err)
	}

	// A full path to the wrapper is still detected (matched by basename).
	pathCMC := "[decoder]\nname=ft8\ncmd=/usr/local/bin/common-mode-cancelling jt9 --ft8 {wav}\n" +
		"parser=jt9\nperiod=15\ncapture=13.5\n"
	if _, err := parseConfig(write(board + pathCMC + chCfg)); err == nil {
		t.Fatal("full-path common-mode-cancelling without iq should fail")
	}

	// A plain wav-to-decoder config is unaffected by the new check (default audio OK).
	wtd := "[decoder]\nname=ft8\ncmd=wav-to-decoder jt9 --ft8 {wav}\n" +
		"parser=jt9\nperiod=15\ncapture=13.5\n"
	if _, err := parseConfig(write(board + wtd + chCfg)); err != nil {
		t.Fatalf("wav-to-decoder config should be unaffected: %v", err)
	}
}
