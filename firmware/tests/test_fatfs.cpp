#include "doctest/doctest.h"
#include "hal/flashfs.hpp"

#include <string_view>

TEST_CASE("fatfs: format, write a file, read it back (RAM disk)") {
  hal::fs_format();
  REQUIRE(hal::fs_mount());
  REQUIRE(hal::fs_append("MAP_001.CSV", "hello,world\n"));
  char buf[64];
  std::size_t len = 0;
  REQUIRE(hal::fs_read("MAP_001.CSV", buf, sizeof buf, len));
  CHECK(std::string_view(buf, len) == "hello,world\n");
}

TEST_CASE("fatfs: streaming write (open once / write many / close) matches content") {
  hal::fs_format();
  REQUIRE(hal::fs_mount());
  REQUIRE(hal::fs_open_write("HOLD001.CSV"));
  REQUIRE(hal::fs_stream_write("a,b,c\n"));
  REQUIRE(hal::fs_stream_write("1,2,3\n"));
  REQUIRE(hal::fs_stream_write("4,5,6\n"));
  REQUIRE(hal::fs_close_write());
  char buf[64];
  std::size_t len = 0;
  REQUIRE(hal::fs_read("HOLD001.CSV", buf, sizeof buf, len));
  CHECK(std::string_view(buf, len) == "a,b,c\n1,2,3\n4,5,6\n");
}

TEST_CASE("fatfs: fs_open_write truncates a pre-existing file (fresh each run)") {
  hal::fs_format();
  REQUIRE(hal::fs_mount());
  REQUIRE(hal::fs_append("HOLD002.CSV", "STALE-OLD-RUN-DATA\n"));
  REQUIRE(hal::fs_open_write("HOLD002.CSV")); // CREATE_ALWAYS -> truncate
  REQUIRE(hal::fs_stream_write("fresh\n"));
  REQUIRE(hal::fs_close_write());
  char buf[64];
  std::size_t len = 0;
  REQUIRE(hal::fs_read("HOLD002.CSV", buf, sizeof buf, len));
  CHECK(std::string_view(buf, len) == "fresh\n"); // no stale bytes
}

TEST_CASE("fatfs: fs_exists reflects file presence") {
  hal::fs_format();
  REQUIRE(hal::fs_mount());
  CHECK_FALSE(hal::fs_exists("HOLD001.CSV"));
  REQUIRE(hal::fs_append("HOLD001.CSV", "x\n"));
  CHECK(hal::fs_exists("HOLD001.CSV"));
  CHECK_FALSE(hal::fs_exists("HOLD002.CSV"));
}

TEST_CASE("fatfs: fs_next_free_name skips existing files (no overwrite across reboot)") {
  hal::fs_format();
  REQUIRE(hal::fs_mount());
  char name[13];
  // Fresh volume -> first slot.
  CHECK(std::string_view(hal::fs_next_free_name(name, sizeof name, "HOLD%03u.CSV")) ==
        "HOLD001.CSV");
  // Occupy 001 and 002 -> next free is 003 (survives a reboot; a RAM counter would
  // reset to 1 and clobber HOLD001).
  REQUIRE(hal::fs_append("HOLD001.CSV", "a\n"));
  REQUIRE(hal::fs_append("HOLD002.CSV", "b\n"));
  CHECK(std::string_view(hal::fs_next_free_name(name, sizeof name, "HOLD%03u.CSV")) ==
        "HOLD003.CSV");
  // A different prefix is counted independently.
  CHECK(std::string_view(hal::fs_next_free_name(name, sizeof name, "MAP_%03u.CSV")) ==
        "MAP_001.CSV");
}
