// Copyright 2021 Milan Vukov
#include "gmock/gmock.h"
#include "gtest/gtest.h"

TEST(SimpleTests, WhenFooBarGiven_EnsureStartsWithFoo) {
  EXPECT_THAT("FooBar", testing::StartsWith("Foo"));
}
