#pragma once

#include <exception>
#include <string>

#define PHOTOCALIB_THROW(text) \
  throw ::photocalib::Exception(__LINE__, __FILE__, text)

#define PHOTOCALIB_ASSERT_MSG(cond, text) \
  if (!(cond)) PHOTOCALIB_THROW(text)

#define PHOTOCALIB_ASSERT(cond) \
  PHOTOCALIB_ASSERT_MSG(cond, "assertion failed: " #cond)

#ifdef NDEBUG
#define PHOTOCALIB_DEBUG_MSG(cond, text)
#define PHOTOCALIB_DEBUG(cond)
#else
#define PHOTOCALIB_DEBUG_MSG PHOTOCALIB_ASSERT_MSG
#define PHOTOCALIB_DEBUG PHOTOCALIB_ASSERT
#endif

namespace photocalib
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

} // namespace photocalib