// fcfb_stream.h -- FPGA->host selected-bin stream contract (M2).
//
// CLEAN-ROOM: written from scratch for the fcfb project. No ka9q-radio and no
// npapi RE code is used here; both are behavioural/performance references only.
//
// The stream is the byte-exact egress format the FPGA Stage-1 will produce and
// the host Stage-2 consumes. For M2 it is read from a file (or loopback); the
// jumbo-UDP / zero-copy transport (PLAN §6) is layered on in M4. Little-endian.
//
//   Header (48 B for v1, 52 B for v2):
//     char magic[8] = "FCFBv1\0\0"
//     u32  version              (1 = int16, 2 = int24)
//     u32  N                    analysis FFT size (4096)
//     f64  fs                   ADC sample rate (Hz)
//     u32  k0                   first absolute bin of the kept run
//     u32  W                    number of bins in the run
//     u32  adc_mask             bit0 = ADC0 (A), bit1 = ADC1 (B)
//     u32  nblocks              number of block records that follow
//     f64  bin_scale            complex_bin = (iN + j*iN) * bin_scale
//     u32  bin_width            v2 ONLY: on-wire I/Q component bits (16 or 24)
//   then nblocks * BlockRecord:
//     u64  seq                  block sequence / sample-index tag
//     for each selected ADC (A before B): W * { iN I, iN Q }  (bins k0..k0+W-1),
//       each component bin_width/8 little-endian signed bytes, and the record
//       zero-padded up to a 32-bit-word boundary (int24 single-ADC odd W only).

#ifndef FCFB_STREAM_H
#define FCFB_STREAM_H

#include <cstdint>
#include <cstring>
#include <complex>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fcfb {

constexpr char     kMagic[8]  = {'F', 'C', 'F', 'B', 'v', '1', 0, 0};
constexpr uint32_t kVersion   = 1;
constexpr uint32_t kAdc0Bit   = 0x1;   // A
constexpr uint32_t kAdc1Bit   = 0x2;   // B

struct StreamHeader {
    uint32_t version;
    uint32_t N;
    double   fs;
    uint32_t k0;
    uint32_t W;
    uint32_t adc_mask;
    uint32_t nblocks;
    double   bin_scale;
    uint32_t bin_width = 16;      // v2 only; v1 streams are int16
};

// Parsed stream: for each selected ADC, a run of W bins x nblocks complex bins
// (already dequantised to physical units). adc[a] present iff bit a set.
struct Stream {
    StreamHeader hdr{};
    // run[a][j] is the length-nblocks time series of bin (k0+j) for ADC a.
    // Only entries whose adc_mask bit is set are populated.
    std::vector<std::vector<std::vector<std::complex<double>>>> run{2};
    std::vector<uint64_t> seq;

    double binw() const { return hdr.fs / hdr.N; }
    bool has(uint32_t adc_bit) const { return hdr.adc_mask & adc_bit; }
};

inline uint32_t rd_u32(const uint8_t* p) { uint32_t v; std::memcpy(&v, p, 4); return v; }
inline uint64_t rd_u64(const uint8_t* p) { uint64_t v; std::memcpy(&v, p, 8); return v; }
inline double   rd_f64(const uint8_t* p) { double   v; std::memcpy(&v, p, 8); return v; }

// Little-endian signed integer of `nbytes` (2 or 3) from a byte pointer.
inline int32_t rd_signed_le(const uint8_t* p, uint32_t nbytes) {
    uint32_t u = 0;
    for (uint32_t i = 0; i < nbytes; ++i) u |= uint32_t(p[i]) << (8 * i);
    uint32_t sbit = 1u << (8 * nbytes - 1);          // sign-extend
    return int32_t((u ^ sbit) - sbit);
}

// Read + dequantise an entire stream file. Assumes a little-endian host.
// Handles both v1 (48-B header, int16 bins) and v2 (52-B header, int24 bins).
inline Stream read_stream(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("cannot open stream: " + path);

    uint8_t h[52];
    if (!f.read(reinterpret_cast<char*>(h), 48))
        throw std::runtime_error("short header");
    if (std::memcmp(h, kMagic, 8) != 0)
        throw std::runtime_error("bad magic");

    Stream s;
    s.hdr.version   = rd_u32(h + 8);
    s.hdr.N         = rd_u32(h + 12);
    s.hdr.fs        = rd_f64(h + 16);
    s.hdr.k0        = rd_u32(h + 24);
    s.hdr.W         = rd_u32(h + 28);
    s.hdr.adc_mask  = rd_u32(h + 32);
    s.hdr.nblocks   = rd_u32(h + 36);
    s.hdr.bin_scale = rd_f64(h + 40);
    if (s.hdr.version == 2) {
        if (!f.read(reinterpret_cast<char*>(h + 48), 4))
            throw std::runtime_error("short v2 header tail");
        s.hdr.bin_width = rd_u32(h + 48);
    } else if (s.hdr.version == kVersion) {
        s.hdr.bin_width = 16;
    } else {
        throw std::runtime_error("unsupported stream version");
    }
    const uint32_t bb = s.hdr.bin_width / 8;          // component bytes (2 or 3)

    const uint32_t W = s.hdr.W, NB = s.hdr.nblocks;
    const double sc = s.hdr.bin_scale;
    const uint32_t bits[2] = {kAdc0Bit, kAdc1Bit};
    uint32_t npres = 0;
    for (int a = 0; a < 2; ++a)
        if (s.hdr.adc_mask & bits[a]) {
            s.run[a].assign(W, std::vector<std::complex<double>>(NB));
            ++npres;
        }
    s.seq.resize(NB);

    const uint32_t payload = npres * W * bb * 2;
    const uint32_t pad = (4 - (payload & 3)) & 3;     // to 32-bit-word boundary
    std::vector<uint8_t> run(W * bb * 2);
    uint8_t padbuf[4];
    for (uint32_t m = 0; m < NB; ++m) {
        uint8_t sq[8];
        if (!f.read(reinterpret_cast<char*>(sq), 8))
            throw std::runtime_error("short block seq");
        s.seq[m] = rd_u64(sq);
        for (int a = 0; a < 2; ++a) {
            if (!(s.hdr.adc_mask & bits[a])) continue;
            if (!f.read(reinterpret_cast<char*>(run.data()),
                        std::streamsize(run.size())))
                throw std::runtime_error("short block payload");
            for (uint32_t j = 0; j < W; ++j) {
                double i = rd_signed_le(&run[(2 * j) * bb], bb) * sc;
                double q = rd_signed_le(&run[(2 * j + 1) * bb], bb) * sc;
                s.run[a][j][m] = {i, q};
            }
        }
        if (pad && !f.read(reinterpret_cast<char*>(padbuf), pad))
            throw std::runtime_error("short block pad");
    }
    return s;
}

}  // namespace fcfb
#endif  // FCFB_STREAM_H
