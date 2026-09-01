#pragma once

#include <cstddef>
#include <string_view>

// Portable FatFs facade shared by the host (RAM-disk) and pico (flash) builds.
// SDK-free: the concrete storage lives in a diskio_*.cpp backend linked per
// target. 8.3 names only (LFN disabled in ffconf.h), so pass names like
// "MAP_001.CSV".
namespace hal {

// Create a fresh FAT volume on the default drive. Returns true on success.
bool fs_format();

// Mount the default drive. Returns true on success.
bool fs_mount();

// Append `data` to `name`, creating the file if it does not exist. Re-opens and
// syncs the file on every call -- fine for a few writes, O(n^2) for thousands
// (use the streaming API below for bulk result flushes).
bool fs_append(const char* name, std::string_view data);

// Streaming write: open `name` FRESH (create/truncate) and keep it open for a
// run of fs_stream_write() calls, then fs_close_write() to sync + close. One
// open + buffered writes + one sync, vs. fs_append's open/seek/sync per call --
// the difference between a fast flush and one so slow it starves USB. Only one
// stream may be open at a time.
bool fs_open_write(const char* name);
bool fs_stream_write(std::string_view data);
bool fs_close_write();

// Read up to `cap` bytes of `name` into `buf`; sets `len` to bytes read.
bool fs_read(const char* name, char* buf, std::size_t cap, std::size_t& len);

// True if `name` exists on the mounted volume.
bool fs_exists(const char* name);

// Format `fmt` (a printf format taking one unsigned, e.g. "HOLD%03u.CSV") with the
// LOWEST N in [1, 999] whose resulting 8.3 name does not yet exist, writing it into
// `out` (cap >= 13) and returning `out`. Scans the live FS so result filenames
// never collide with (overwrite) existing files -- a RAM counter resets to 1 on
// reboot and would clobber the file it wrote last session. If all 999 are taken the
// last is reused.
const char* fs_next_free_name(char* out, std::size_t cap, const char* fmt);

} // namespace hal
