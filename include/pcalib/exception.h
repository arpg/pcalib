#pragma once

#include <exception>
#include <string>

#define PCALIB_THROW(text) \
  throw ::pcalib::Exception(__LINE__, __FILE__, text)

#define PCALIB_ASSERT_MSG(cond, text) \
  if (!(cond)) PCALIB_THROW(text)

#define PCALIB_ASSERT(cond) \
  PCALIB_ASSERT_MSG(cond, "assertion failed: " #cond)

#ifdef NDEBUG
#define PCALIB_DEBUG_MSG(cond, text) if (!(cond)) {}
#define PCALIB_DEBUG(cond) if (!(cond)) {}
#else
#define PCALIB_DEBUG_MSG PCALIB_ASSERT_MSG
#define PCALIB_DEBUG PCALIB_ASSERT
#endif

namespace pcalib
{

class Exception : public std::exception
{
  public:

    Exception(int line, const std::string& file, const std::string& text) :
      line_(line),
      file_(file),
      text_(text)
    {
      Initialize();
    }

    inline int line() const
    {
      return line_;
    }

    inline const std::string& file() const
    {
      return file_;
    }

    inline const std::string& text() const
    {
      return text_;
    }

    inline const char* what() const throw() override
    {
      return what_.c_str();
    }

  private:

    void Initialize()
    {
      what_ = file_ + "(" + std::to_string(line_) + "): " + text_;
    }

  protected:

    int line_;

    std::string file_;

    std::string text_;

    std::string what_;
};

} // namespace pcalib