#include <gtest/gtest.h>

bool unused_func() { return false; }

TEST(test_1, unused_func) { EXPECT_EQ(true, unused_func()); }

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}