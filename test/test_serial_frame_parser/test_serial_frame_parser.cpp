#include <gtest/gtest.h>

#include "helpers/SerialFrameParser.h"

namespace {

using Parser = SerialFrameParser<176>;

TEST(SerialFrameParserTest, AcceptsCompleteFrameAndPayloadMarker) {
  Parser parser('<', 100);

  EXPECT_EQ(Parser::NEED_MORE, parser.feed('<', 0));
  EXPECT_EQ(Parser::NEED_MORE, parser.feed(3, 1));
  EXPECT_EQ(Parser::NEED_MORE, parser.feed(0, 2));
  EXPECT_EQ(Parser::NEED_MORE, parser.feed(0x01, 3));
  EXPECT_EQ(Parser::NEED_MORE, parser.feed('<', 4));
  EXPECT_EQ(Parser::FRAME_READY, parser.feed(0x02, 5));

  ASSERT_EQ(3U, parser.frameLength());
  EXPECT_EQ(0x01, parser.frame()[0]);
  EXPECT_EQ('<', parser.frame()[1]);
  EXPECT_EQ(0x02, parser.frame()[2]);
}

TEST(SerialFrameParserTest, RejectsOversizedLengthImmediately) {
  Parser parser('<', 100);

  EXPECT_EQ(Parser::NEED_MORE, parser.feed('<', 0));
  EXPECT_EQ(Parser::NEED_MORE, parser.feed(177, 1));
  EXPECT_EQ(Parser::INVALID_FRAME, parser.feed(0, 2));

  EXPECT_EQ(Parser::NEED_MORE, parser.feed('<', 3));
  EXPECT_EQ(Parser::NEED_MORE, parser.feed(1, 4));
  EXPECT_EQ(Parser::NEED_MORE, parser.feed(0, 5));
  EXPECT_EQ(Parser::FRAME_READY, parser.feed(0x0A, 6));
}

TEST(SerialFrameParserTest, RecoversMarkerThatInvalidatesHeader) {
  Parser parser('<', 100);

  EXPECT_EQ(Parser::NEED_MORE, parser.feed('<', 0));
  EXPECT_EQ(Parser::NEED_MORE, parser.feed(0xFF, 1));
  EXPECT_EQ(Parser::INVALID_FRAME, parser.feed('<', 2));

  // The last '<' was retained as the next frame's marker.
  EXPECT_EQ(Parser::NEED_MORE, parser.feed(1, 3));
  EXPECT_EQ(Parser::NEED_MORE, parser.feed(0, 4));
  EXPECT_EQ(Parser::FRAME_READY, parser.feed(0x83, 5));
  EXPECT_EQ(0x83, parser.frame()[0]);
}

TEST(SerialFrameParserTest, TimesOutPartialPayloadAndResynchronizes) {
  Parser parser('<', 50);

  EXPECT_EQ(Parser::NEED_MORE, parser.feed('<', 0));
  EXPECT_EQ(Parser::NEED_MORE, parser.feed(3, 1));
  EXPECT_EQ(Parser::NEED_MORE, parser.feed(0, 2));
  EXPECT_EQ(Parser::NEED_MORE, parser.feed(0xAA, 3));

  // A marker after the inter-byte timeout starts a clean frame.
  EXPECT_EQ(Parser::INVALID_FRAME, parser.feed('<', 100));
  EXPECT_EQ(Parser::NEED_MORE, parser.feed(1, 101));
  EXPECT_EQ(Parser::NEED_MORE, parser.feed(0, 102));
  EXPECT_EQ(Parser::FRAME_READY, parser.feed(0x00, 103));
  EXPECT_EQ(0x00, parser.frame()[0]);
}

TEST(SerialFrameParserTest, AcceptsMaximumLength) {
  Parser parser('<', 100);

  EXPECT_EQ(Parser::NEED_MORE, parser.feed('<', 0));
  EXPECT_EQ(Parser::NEED_MORE, parser.feed(176, 1));
  EXPECT_EQ(Parser::NEED_MORE, parser.feed(0, 2));
  for (unsigned i = 0; i < 175; ++i) {
    EXPECT_EQ(Parser::NEED_MORE, parser.feed((uint8_t)i, 3 + i));
  }
  EXPECT_EQ(Parser::FRAME_READY, parser.feed(0xAF, 178));
  EXPECT_EQ(176U, parser.frameLength());
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
