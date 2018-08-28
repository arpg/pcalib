#include <gtest/gtest.h>
#include <photocalib/exception.h>
#include <photocalib/polynomial_response.h>

namespace photocalib
{
namespace testing
{

TEST(PolynomialResponse, Constructor)
{
  PolynomialResponse a;
  ASSERT_EQ(1, a.degree());
  ASSERT_EQ(1.0, a[0]);

  PolynomialResponse b(1);
  ASSERT_EQ(1, b.degree());
  ASSERT_EQ(1.0, b[0]);

  PolynomialResponse c(2);
  ASSERT_EQ(2, c.degree());
  ASSERT_EQ(1.0, c[0]);
  ASSERT_EQ(0.0, c[1]);

  PolynomialResponse d(4);
  ASSERT_EQ(4, d.degree());
  ASSERT_EQ(1.0, d[0]);
  ASSERT_EQ(0.0, d[1]);
  ASSERT_EQ(0.0, d[2]);
  ASSERT_EQ(0.0, d[3]);

  ASSERT_THROW(PolynomialResponse(0), Exception);
  ASSERT_THROW(PolynomialResponse(-1), Exception);
}

TEST(PolynomialResponse, Degree)
{
  PolynomialResponse response;
  response.set_degree(4);

  ASSERT_EQ(4, response.degree());
  ASSERT_EQ(4, response.coeffs().size());
  ASSERT_EQ(1.0, response[0]);
  ASSERT_EQ(0.0, response[1]);
  ASSERT_EQ(0.0, response[2]);
  ASSERT_EQ(0.0, response[3]);

  response[0] = 2.1;
  response[1] = 1.3;
  response.set_degree(2);

  ASSERT_EQ(2, response.degree());
  ASSERT_EQ(2, response.coeffs().size());
  ASSERT_EQ(2.1, response[0]);
  ASSERT_EQ(1.3, response[1]);

  response[0] = -1.7;
  response[1] =  3.2;
  response.set_degree(3);

  ASSERT_EQ(3, response.degree());
  ASSERT_EQ(3, response.coeffs().size());
  ASSERT_EQ(-1.7, response[0]);
  ASSERT_EQ( 3.2, response[1]);
  ASSERT_EQ( 0.0, response[2]);

#ifndef NDEBUG
  ASSERT_THROW(response.set_degree(0), Exception);
  ASSERT_THROW(response.set_degree(-1), Exception);
#endif
}

TEST(PolynomialResponse, Coeffs)
{
  PolynomialResponse response;
  response.set_degree(4);

  ASSERT_EQ(4, response.coeffs().size());
  ASSERT_EQ(1.0, response.coeffs()[0]);
  ASSERT_EQ(0.0, response.coeffs()[1]);
  ASSERT_EQ(0.0, response.coeffs()[2]);
  ASSERT_EQ(0.0, response.coeffs()[3]);

  response.set_coeffs(Eigen::Vector4d(0.1, -2.7, 5.6, -8.9));
  ASSERT_EQ(4, response.coeffs().size());
  ASSERT_EQ( 0.1, response.coeffs()[0]);
  ASSERT_EQ(-2.7, response.coeffs()[1]);
  ASSERT_EQ( 5.6, response.coeffs()[2]);
  ASSERT_EQ(-8.9, response.coeffs()[3]);

  ASSERT_THROW(response.set_coeffs(Eigen::Vector2d(0, 0)), Exception);
  ASSERT_THROW(response.set_coeffs(Eigen::Vector3d(0, 0, 0)), Exception);
}

TEST(PolynomialResponse, Indexing)
{
  PolynomialResponse response;
  response.set_degree(4);

  response[0] =  1.3;
  response[1] = -0.7;
  response[2] =  4.6;
  response[3] = -9.0;

  ASSERT_EQ( 1.3, response.coeffs()[0]);
  ASSERT_EQ(-0.7, response.coeffs()[1]);
  ASSERT_EQ( 4.6, response.coeffs()[2]);
  ASSERT_EQ(-9.0, response.coeffs()[3]);

  response.set_coeffs(Eigen::Vector4d(0.1, -2.3, 7.2, -8.4));

  ASSERT_EQ( 0.1, response[0]);
  ASSERT_EQ(-2.3, response[1]);
  ASSERT_EQ( 7.2, response[2]);
  ASSERT_EQ(-8.4, response[3]);

#ifndef NDEBUG
  ASSERT_THROW(response[-1], Exception);
  ASSERT_THROW(response[response.degree()], Exception);
#endif
}

TEST(PolynomialResponse, Response)
{
  std::vector<double> values;
  values.push_back( 0.0000);
  values.push_back( 1.0000);
  values.push_back( 0.5000);
  values.push_back( 0.7512);
  values.push_back(-3.3771);
  values.push_back( 1.9382);

  PolynomialResponse response;
  response.set_degree(4);

  for (double value : values)
  {
    ASSERT_NEAR(value, response(value), 1E-8);
  }

  response.set_degree(2);
  response[0] =  2.7;
  response[1] = -1.2;

  for (double value : values)
  {
    double expected = 0.0;

    for (int i = 0; i < response.degree(); ++i)
    {
      expected += std::pow(value, i + 1) * response[i];
    }

    ASSERT_NEAR(expected, response(value), 1E-8);
  }

  response.set_degree(4);
  response[0] =  1.3;
  response[1] = -0.7;
  response[2] =  4.6;
  response[3] = -9.0;

  for (double value : values)
  {
    double expected = 0.0;

    for (int i = 0; i < response.degree(); ++i)
    {
      expected += std::pow(value, i + 1) * response[i];
    }

    ASSERT_NEAR(expected, response(value), 1E-8);
  }

#ifndef NDEBUG
  const double nan = std::numeric_limits<double>::quiet_NaN();
  ASSERT_THROW(response(nan), Exception);
#endif
}

} // namespace testing

} // namespace photocalib