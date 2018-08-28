#pragma once

namespace photocalib
{

class Response
{
  public:

    virtual double operator()(double value) const = 0;
};

} // namespace photocalib