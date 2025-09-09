target_sources(${UNIT_TESTS_APPLICATION}
    PRIVATE
    test_abw_data.h
    test_data.h

    big_snake_parse_test.cc
    joint_model_test.cc
    naive_test.cc
    slice_test.cc
    snake_test.cc
)
