px4_add_module(
	MODULE modules__remaining_range
	MAIN remaining_range
	SRCS
		remaining_range.cpp
	DEPENDS
		px4_work_queue
	)
