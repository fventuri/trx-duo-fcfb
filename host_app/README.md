# host_app

The host (PC) side of the split FCFB channelizer.

- **[`gofcfb/`](gofcfb/)** — the production host applications (`fcfbfarm`,
  `fcfbhpsdr`, `fcfbinfo`): a zero-dependency Go implementation of the board
  transport, channel synthesis, and the openHPSDR Protocol-2 emitter. See
  [`gofcfb/README.md`](gofcfb/README.md) for building, cross-compiling, and running.
- **`fixtures/`** — small captured board streams used by the Go tests
  (`go test ./...`).
