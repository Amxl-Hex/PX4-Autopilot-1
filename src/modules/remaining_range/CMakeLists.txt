px4_add_module(
	MODULE modules__remaining_range
	MAIN remaining_range
	STACK_MAIN 2000
	SRCS
		remaining_range.cpp
	DEPENDS
		px4_config
	)
