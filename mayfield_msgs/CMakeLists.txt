cmake_minimum_required(VERSION 2.8.3)
project(mayfield_msgs)


add_compile_options(-std=c++11)


find_package(catkin REQUIRED
        COMPONENTS
        message_generation
        std_msgs
        actionlib_msgs
        geometry_msgs
        nav_msgs
        )

add_message_files(
        FILES
        KeyValue.msg
        KeyValueArray.msg
        NodeStatus.msg
        NavigateActionFeedback.msg
        NavigateActionGoal.msg
        NavigateAction.msg
        NavigateActionResult.msg
        NavigateFeedback.msg
        NavigateGoal.msg
        NavigateResult.msg
)

add_service_files(
        FILES
        SetString.srv
        StringExchange.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
        actionlib_msgs
        geometry_msgs
        nav_msgs
)

catkin_package(
        CATKIN_DEPENDS
        std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
