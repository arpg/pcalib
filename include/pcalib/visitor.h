#pragma once

namespace pcalib
{

template <typename T>
class Visitor
{
  public:

    virtual void Visit(const T&) = 0;
};

template <typename Visitor>
class Visitable
{
  public:

    virtual void Accept(Visitor& visitor) const;
};


} // namespace pcalib