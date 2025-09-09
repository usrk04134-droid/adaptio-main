target_sources(${UNIT_TESTS_APPLICATION}
    PRIVATE
    test_data.h

    big_snake_test.cc
    big_snake_yaml_test.cc
    joint_model_test.cc
    naive_test.cc
    slice_test.cc
    snake_test.cc
)
