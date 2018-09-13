#include <gtest/gtest.h>
#include <pcalib/exception.h>

namespace pcalib
{
namespace testing
{

TEST(Exception, Constructor)
{
  const int expected_line = __LINE__;
  const std::string expected_file = __FILE__;
  const std::string expected_text = "expected text";

  const std::string expected_what = expected_file + "(" +
      std::to_string(expected_line) + "): " + expected_text;

  const Exception exception(expected_line, expected_file, expected_text);
  ASSERT_EQ(expected_line, exception.line());
  ASSERT_EQ(expected_file, exception.file());
  ASSERT_EQ(expected_text, exception.text());
  ASSERT_EQ(expected_what, exception.what());
}

TEST(Exception, Throw)
{
  int expected_line;
  const std::string expected_file = __FILE__;
  const std::string expected_text = "expected text";
  bool thrown = false;

  try
  {
    expected_line = __LINE__ + 1;
    PHOTOCALIB_THROW(expected_text);
  }
  catch (const Exception& exception)
  {
    const std::string expected_what = expected_file + "(" +
        std::to_string(expected_line) + "): " + expected_text;

    ASSERT_EQ(expected_line, exception.line());
    ASSERT_EQ(expected_file, exception.file());
    ASSERT_EQ(expected_text, exception.text());
    ASSERT_EQ(expected_what, exception.what());
    thrown = true;
  }

  ASSERT_TRUE(thrown);
}

TEST(Exception, AssertMessage)
{
  int expected_line;
  const std::string expected_file = __FILE__;
  const std::string expected_text = "expected text";
  bool thrown = false;

  try
  {
    expected_line = __LINE__ + 1;
    PHOTOCALIB_ASSERT_MSG(0 > 1, expected_text);
  }
  catch (const Exception& exception)
  {
    const std::string expected_what = expected_file + "(" +
        std::to_string(expected_line) + "): " + expected_text;

    ASSERT_EQ(expected_line, exception.line());
    ASSERT_EQ(expected_file, exception.file());
    ASSERT_EQ(expected_text, exception.text());
    ASSERT_EQ(expected_what, exception.what());
    thrown = true;
  }

  ASSERT_TRUE(thrown);
  ASSERT_NO_THROW(PHOTOCALIB_ASSERT_MSG(0 < 1, ""));
}

TEST(Exception, Assert)
{
  int expected_line;
  const std::string expected_file = __FILE__;
  const std::string expected_text = "assertion failed: 0 > 1";
  bool thrown = false;

  try
  {
    expected_line = __LINE__ + 1;
    PHOTOCALIB_ASSERT(0 > 1);
  }
  catch (const Exception& exception)
  {
    const std::string expected_what = expected_file + "(" +
        std::to_string(expected_line) + "): " + expected_text;

    ASSERT_EQ(expected_line, exception.line());
    ASSERT_EQ(expected_file, exception.file());
    ASSERT_EQ(expected_text, exception.text());
    ASSERT_EQ(expected_what, exception.what());
    thrown = true;
  }

  ASSERT_TRUE(thrown);
  ASSERT_NO_THROW(PHOTOCALIB_ASSERT(0 < 1));
}

TEST(Exception, DebugMessage)
{
  int expected_line;
  const std::string expected_file = __FILE__;
  const std::string expected_text = "expected text";
  bool thrown = false;

  try
  {
    expected_line = __LINE__ + 1;
    PHOTOCALIB_DEBUG_MSG(0 > 1, expected_text);
  }
  catch (const Exception& exception)
  {
    const std::string expected_what = expected_file + "(" +
        std::to_string(expected_line) + "): " + expected_text;

    ASSERT_EQ(expected_line, exception.line());
    ASSERT_EQ(expected_file, exception.file());
    ASSERT_EQ(expected_text, exception.text());
    ASSERT_EQ(expected_what, exception.what());
    thrown = true;
  }

#ifdef NDEBUG
  ASSERT_FALSE(thrown);
#else
  ASSERT_TRUE(thrown);
#endif

  ASSERT_NO_THROW(PHOTOCALIB_DEBUG_MSG(0 < 1, ""));
}

TEST(Exception, Debug)
{
  int expected_line;
  const std::string expected_file = __FILE__;
  const std::string expected_text = "assertion failed: 0 > 1";
  bool thrown = false;

  try
  {
    expected_line = __LINE__ + 1;
    PHOTOCALIB_DEBUG(0 > 1);
  }
  catch (const Exception& exception)
  {
    const std::string expected_what = expected_file + "(" +
        std::to_string(expected_line) + "): " + expected_text;

    ASSERT_EQ(expected_line, exception.line());
    ASSERT_EQ(expected_file, exception.file());
    ASSERT_EQ(expected_text, exception.text());
    ASSERT_EQ(expected_what, exception.what());
    thrown = true;
  }

#ifdef NDEBUG
  ASSERT_FALSE(thrown);
#else
  ASSERT_TRUE(thrown);
#endif

  ASSERT_NO_THROW(PHOTOCALIB_DEBUG(0 < 1));
}

} // namespace testing

} // namespace pcalib