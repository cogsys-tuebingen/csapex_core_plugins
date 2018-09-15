#include <csapex_testing/csapex_test_case.h>
#include <csapex/utility/singleton.hpp>

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);

    auto res = RUN_ALL_TESTS();
    return res;
}


