#include <string>
#include <fstream>
#include <vector>

#include <message_filters/subscriber.h>  
#include <message_filters/synchronizer.h>  
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/seam.h>

#include <sensor_msgs/msg/joint_state.hpp>

#include "signal.h"
#include "helper.h"

using namespace message_filters;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;
using std::placeholders::_5;
using std::placeholders::_6;
using std::placeholders::_7;
using std::placeholders::_8;
using std::placeholders::_9;

// int BOUND_Between_s;
int count_apr = 0;
int count_seam = 0;
int suc_apr = 0;
int suc_seam = 0;

namespace synchronizer
{

#define buffer_size 10000000

class SubscriberTopic2
    : public rclcpp::Node
{
    ofstream& outfile_apr_;
    ofstream& outfile_seam_;
    ofstream& outfile_computetime_apr;
    ofstream& outfile_computetime_seam;
    double minus_apr;
    double minus_seam;

    public:
    SubscriberTopic2(int period1, int period2, int x_ms, int length_ms, int BOUND_Between_ms, std::ofstream& outfile_apr, std::ofstream& outfile_seam, std::ofstream& outfile_ct_apr, std::ofstream& outfile_ct_seam) :
        Node("subscriber_topic2"),  outfile_apr_(outfile_apr), outfile_seam_(outfile_seam), outfile_computetime_apr(outfile_ct_apr), outfile_computetime_seam(outfile_ct_seam), 
        apr_sync_(buffer_size), seam_sync_(), period1_(period1), period2_(period2), x_ms_(x_ms), length_ms_(length_ms), Bound_between_ms_(BOUND_Between_ms)
    {
        callback_group_subscriber_ = this->create_callback_group(
        rclcpp::callback_group::CallbackGroupType::Reentrant);

        // Each of these callback groups is basically a thread
        // Everything assigned to one of them gets bundled into the same thread
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_subscriber_;

        topic1_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic1", 300, 
                                                                        std::bind(&SubscriberTopic2::callback1, this, _1), sub_opt);
        topic2_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic2", 300, 
                                                                        std::bind(&SubscriberTopic2::callback2, this, _1), sub_opt);

        apr_sync_.registerCallback(std::bind(&SubscriberTopic2::apr_callback, this, _1, _2));
        seam_sync_.registerCallback(std::bind(&SubscriberTopic2::seam_callback, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Period 1: %d, Period 2: %d", period1, period2);

        apr_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        apr_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);

        seam_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        seam_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);

        apr_sync_.setAgePenalty(0);
        // seam_sync_.setAgePenalty(0);
        seam_sync_.setBOUND(x_ms);

        suc_apr = 0;
        suc_seam = 0;

        nanosec = Bound_between_ms_ * 1000000;
        rclcpp::Duration duration(nanosec);
        BOUND_between_ = duration;

        stop_timer_ = this->create_wall_timer(std::chrono::milliseconds(length_ms_), std::bind(&SubscriberTopic2::stop_receive, this));
    }

    ~SubscriberTopic2()
    {
    }

    private:

    void stop_receive()
    {
        RCLCPP_INFO(this->get_logger(), "Time is up! This subscriber has stopped receiving!");
        kill(getpid(),SIGINT);
    }

    void apr_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2)
    {
        double topic1_timestamp = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double topic2_timestamp = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        outfile_apr_ << topic1_timestamp << " " << topic2_timestamp << endl;
        double time_disparity = cal_time_disparity(2, topic1_timestamp, topic2_timestamp);

        count_apr++;

        if (apr_first_sync_)
        {
            apr_pre_sync_time_ = this->now();
            apr_first_sync_ = false;
        }
        else
        {
            apr_this_sync_time_ = this->now();
            if ((time_disparity < x_ms_) && ((apr_this_sync_time_ - apr_pre_sync_time_) < BOUND_between_))
            {
                suc_apr++;
            }
            apr_pre_sync_time_ = apr_this_sync_time_;
        }
    }

    void seam_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2)
    {
        double topic1_timestamp = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double topic2_timestamp = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        outfile_seam_ << topic1_timestamp << " " << topic2_timestamp << endl;
        double time_disparity = cal_time_disparity(2, topic1_timestamp, topic2_timestamp);

        count_seam++;

        if (seam_first_sync_)
        {
            seam_pre_sync_time_ = this->now();
            seam_first_sync_ = false;
        }
        else
        {
            seam_this_sync_time_ = this->now();
            if ((time_disparity < x_ms_) && ((seam_this_sync_time_ - seam_pre_sync_time_) < BOUND_between_))
            {
                suc_seam++;
            }
            seam_pre_sync_time_ = seam_this_sync_time_;
        }
    }

    void callback1(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic0 by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<0>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic0  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic0  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<0>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic0  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback2(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic1  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<1>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic1  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic1  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<1>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic1  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_subscriber_;
    rclcpp::TimerBase::SharedPtr stop_timer_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic1_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic2_sub_;

    typedef Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > AlgSync;
    typedef Synchronizer<sync_policies::SEAM<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > SeamSync;

    AlgSync apr_sync_;
    SeamSync seam_sync_;

    int period1_, period2_;
    int x_ms_;
    int length_ms_;
    rclcpp::Time apr_pre_sync_time_;
    rclcpp::Time apr_this_sync_time_;
    rclcpp::Time seam_pre_sync_time_;
    rclcpp::Time seam_this_sync_time_;
    bool apr_first_sync_ = true;
    bool seam_first_sync_ = true;

    int Bound_between_ms_;
    rclcpp::Duration BOUND_between_ = rclcpp::Duration(0.125, 0);
    int64_t nanosec;
};

class SubscriberTopic3
    : public rclcpp::Node
{
    ofstream& outfile_apr_;
    ofstream& outfile_seam_;
    ofstream& outfile_computetime_apr;
    ofstream& outfile_computetime_seam;
    double minus_apr;
    double minus_seam;

    public:
    SubscriberTopic3(int period1, int period2, int period3, int x_ms, int length_ms, int BOUND_Between_ms, std::ofstream& outfile_apr, std::ofstream& outfile_seam, std::ofstream& outfile_ct_apr, std::ofstream& outfile_ct_seam) :
        Node("subscriber_topic3"), outfile_apr_(outfile_apr), outfile_seam_(outfile_seam), outfile_computetime_apr(outfile_ct_apr), outfile_computetime_seam(outfile_ct_seam), 
        apr_sync_(buffer_size), seam_sync_(), period1_(period1), period2_(period2), period3_(period3), x_ms_(x_ms), length_ms_(length_ms), Bound_between_ms_(BOUND_Between_ms)
    {
        callback_group_subscriber_ = this->create_callback_group(
        rclcpp::callback_group::CallbackGroupType::Reentrant);

        // Each of these callback groups is basically a thread
        // Everything assigned to one of them gets bundled into the same thread
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_subscriber_;

        topic1_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic1", 300, 
                                                                        std::bind(&SubscriberTopic3::callback1, this, _1), sub_opt);

        topic2_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic2", 300, 
                                                                        std::bind(&SubscriberTopic3::callback2, this, _1), sub_opt);

        topic3_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic3", 300, 
                                                                        std::bind(&SubscriberTopic3::callback3, this, _1), sub_opt);

        apr_sync_.registerCallback(std::bind(&SubscriberTopic3::apr_callback, this, _1, _2, _3));
        seam_sync_.registerCallback(std::bind(&SubscriberTopic3::seam_callback, this, _1, _2, _3));

        RCLCPP_INFO(this->get_logger(), "Period 1: %d, Period 2: %d, Period 3: %d", period1, period2, period3);

        apr_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        apr_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        apr_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);

        seam_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        seam_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        seam_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);
        
        apr_sync_.setAgePenalty(0);
        // seam_sync_.setAgePenalty(0);
        seam_sync_.setBOUND(x_ms);

        suc_apr = 0;
        suc_seam = 0;

        nanosec = Bound_between_ms_ * 1000000;
        rclcpp::Duration duration(nanosec);
        BOUND_between_ = duration;

        stop_timer_ = this->create_wall_timer(std::chrono::milliseconds(length_ms_), std::bind(&SubscriberTopic3::stop_receive, this));
    }

    ~SubscriberTopic3()
    {
    }

    private:

    void stop_receive()
    {
        RCLCPP_INFO(this->get_logger(), "Time is up! This subscriber has stopped receiving!");
        kill(getpid(),SIGINT);
    }

    void apr_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3)
    {
        double topic1_timestamp = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double topic2_timestamp = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        double topic3_timestamp = (double)msg3->header.stamp.sec + 1e-9*(double)msg3->header.stamp.nanosec;
        outfile_apr_ << topic1_timestamp << " " << topic2_timestamp << " " << topic3_timestamp << endl;
        
        double time_disparity = cal_time_disparity(3, topic1_timestamp, topic2_timestamp, topic3_timestamp);

        count_apr++;

        if (apr_first_sync_)
        {
            apr_pre_sync_time_ = this->now();
            apr_first_sync_ = false;
        }
        else
        {
            apr_this_sync_time_ = this->now();
            if ((time_disparity < x_ms_) && ((apr_this_sync_time_ - apr_pre_sync_time_) < BOUND_between_))
            {
                suc_apr++;
            }
            apr_pre_sync_time_ = apr_this_sync_time_;
        }
    }

    void seam_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3)
    {
        double topic1_timestamp = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double topic2_timestamp = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        double topic3_timestamp = (double)msg3->header.stamp.sec + 1e-9*(double)msg3->header.stamp.nanosec;
                    
        outfile_seam_ << topic1_timestamp << " " << topic2_timestamp << " " << topic3_timestamp << endl;
        double time_disparity = cal_time_disparity(3, topic1_timestamp, topic2_timestamp, topic3_timestamp);

        count_seam++;

        if (seam_first_sync_)
        {
            seam_pre_sync_time_ = this->now();
            seam_first_sync_ = false;
        }
        else
        {
            seam_this_sync_time_ = this->now();
            if ((time_disparity < x_ms_) && ((seam_this_sync_time_ - seam_pre_sync_time_) < BOUND_between_))
            {
                suc_seam++;
            }
            seam_pre_sync_time_ = seam_this_sync_time_;
        }
    }

    void callback1(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {   
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic0  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<0>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic0  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic0  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<0>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic0  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback2(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic1  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<1>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic1  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic1  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<1>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic1  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback3(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic2  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<2>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic2  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic2  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<2>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic2  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_subscriber_;
    rclcpp::TimerBase::SharedPtr stop_timer_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic1_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic2_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic3_sub_;

    typedef Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > AlgSync;
    typedef Synchronizer<sync_policies::SEAM<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > SeamSync;

    AlgSync apr_sync_;
    SeamSync seam_sync_;

    int period1_, period2_, period3_;
    int x_ms_;
    int length_ms_;
    rclcpp::Time apr_pre_sync_time_;
    rclcpp::Time apr_this_sync_time_;
    rclcpp::Time seam_pre_sync_time_;
    rclcpp::Time seam_this_sync_time_;
    bool apr_first_sync_ = true;
    bool seam_first_sync_ = true;
    
    int Bound_between_ms_;
    rclcpp::Duration BOUND_between_ = rclcpp::Duration(0.125, 0);
    int64_t nanosec;
};

class SubscriberTopic4
    : public rclcpp::Node
{
    ofstream& outfile_apr_;
    ofstream& outfile_seam_;
    ofstream& outfile_computetime_apr;
    ofstream& outfile_computetime_seam;
    double minus_apr;
    double minus_seam;

    public:
    SubscriberTopic4(int period1, int period2, int period3, int period4, int x_ms, int length_ms, int BOUND_Between_ms, std::ofstream& outfile_apr, std::ofstream& outfile_seam, std::ofstream& outfile_ct_apr, std::ofstream& outfile_ct_seam) :
        Node("subscriber_topic4"), outfile_apr_(outfile_apr), outfile_seam_(outfile_seam), outfile_computetime_apr(outfile_ct_apr), outfile_computetime_seam(outfile_ct_seam), 
        apr_sync_(buffer_size), seam_sync_(), 
        period1_(period1), period2_(period2), period3_(period3), period4_(period4), x_ms_(x_ms), length_ms_(length_ms), Bound_between_ms_(BOUND_Between_ms)
    {
        callback_group_subscriber_ = this->create_callback_group(
        rclcpp::callback_group::CallbackGroupType::Reentrant);

        // Each of these callback groups is basically a thread
        // Everything assigned to one of them gets bundled into the same thread
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_subscriber_;

        topic1_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic1", 300, 
                                                                        std::bind(&SubscriberTopic4::callback1, this, _1), sub_opt);

        topic2_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic2", 300, 
                                                                        std::bind(&SubscriberTopic4::callback2, this, _1), sub_opt);

        topic3_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic3", 300, 
                                                                        std::bind(&SubscriberTopic4::callback3, this, _1), sub_opt);

        topic4_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic4", 300, 
                                                                        std::bind(&SubscriberTopic4::callback4, this, _1), sub_opt);

        apr_sync_.registerCallback(std::bind(&SubscriberTopic4::apr_callback, this, _1, _2, _3, _4));
        seam_sync_.registerCallback(std::bind(&SubscriberTopic4::seam_callback, this, _1, _2, _3, _4));

        RCLCPP_INFO(this->get_logger(), "Period 1: %d, Period 2: %d, Period 3: %d, Period 4: %d",
                    period1, period2, period3, period4);

        apr_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        apr_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        apr_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);
        apr_sync_.setInterMessageLowerBound(3, PeriodBase * 0.001 * period4);

        seam_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        seam_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        seam_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);
        seam_sync_.setInterMessageLowerBound(3, PeriodBase * 0.001 * period4);

        apr_sync_.setAgePenalty(0);
        // seam_sync_.setAgePenalty(0);
        seam_sync_.setBOUND(x_ms);

        suc_apr = 0;
        suc_seam = 0;

        nanosec = Bound_between_ms_ * 1000000;
        rclcpp::Duration duration(nanosec);
        BOUND_between_ = duration;

        stop_timer_ = this->create_wall_timer(std::chrono::milliseconds(length_ms_), std::bind(&SubscriberTopic4::stop_receive, this));
    }

    ~SubscriberTopic4()
    {
    }

    private:
    void stop_receive()
    {
        RCLCPP_INFO(this->get_logger(), "Time is up! This subscriber has stopped receiving!");
        kill(getpid(),SIGINT);
    }

    void apr_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4)
    {
        double topic1_timestamp = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double topic2_timestamp = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        double topic3_timestamp = (double)msg3->header.stamp.sec + 1e-9*(double)msg3->header.stamp.nanosec;
        double topic4_timestamp = (double)msg4->header.stamp.sec + 1e-9*(double)msg4->header.stamp.nanosec;
        outfile_apr_ << topic1_timestamp << " " << topic2_timestamp << " " << topic3_timestamp << " " << topic4_timestamp << endl;
        
        double time_disparity = cal_time_disparity(4, topic1_timestamp, topic2_timestamp, topic3_timestamp, topic4_timestamp);
        
        count_apr++;

        if (apr_first_sync_)
        {
            apr_pre_sync_time_ = this->now();
            apr_first_sync_ = false;
        }
        else
        {
            apr_this_sync_time_ = this->now();
            if ((time_disparity < x_ms_) && ((apr_this_sync_time_ - apr_pre_sync_time_) < BOUND_between_))
            {
                suc_apr++;
            }
            apr_pre_sync_time_ = apr_this_sync_time_;
        }
    }

    void seam_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4)
    {
        double topic1_timestamp = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double topic2_timestamp = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        double topic3_timestamp = (double)msg3->header.stamp.sec + 1e-9*(double)msg3->header.stamp.nanosec;
        double topic4_timestamp = (double)msg4->header.stamp.sec + 1e-9*(double)msg4->header.stamp.nanosec;
        
        outfile_seam_ << topic1_timestamp << " " << topic2_timestamp << " " << topic3_timestamp << " " << topic4_timestamp << endl;
        double time_disparity = cal_time_disparity(4, topic1_timestamp, topic2_timestamp, topic3_timestamp, topic4_timestamp);
        
        count_seam++;

        if (seam_first_sync_)
        {
            seam_pre_sync_time_ = this->now();
            seam_first_sync_ = false;
        }
        else
        {
            seam_this_sync_time_ = this->now();
            if ((time_disparity < x_ms_) && ((seam_this_sync_time_ - seam_pre_sync_time_) < BOUND_between_))
            {
                suc_seam++;
            }
            seam_pre_sync_time_ = seam_this_sync_time_;
        }
    }

    void callback1(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {   
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic0  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<0>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic0  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic0  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<0>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic0  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback2(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic1  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<1>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic1  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic1  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<1>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic1  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback3(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic2  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<2>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic2  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic2  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<2>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic2  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback4(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic3  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<3>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic3  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic3  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<3>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic3  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_subscriber_;
    rclcpp::TimerBase::SharedPtr stop_timer_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic1_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic2_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic3_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic4_sub_;

    typedef Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > AlgSync;
    typedef Synchronizer<sync_policies::SEAM<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > SeamSync;

    AlgSync apr_sync_;
    SeamSync seam_sync_;

    int period1_, period2_, period3_, period4_;
    int x_ms_;
    int length_ms_;
    rclcpp::Time apr_pre_sync_time_;
    rclcpp::Time apr_this_sync_time_;
    rclcpp::Time seam_pre_sync_time_;
    rclcpp::Time seam_this_sync_time_;
    bool apr_first_sync_ = true;
    bool seam_first_sync_ = true;
    
    int Bound_between_ms_;
    rclcpp::Duration BOUND_between_ = rclcpp::Duration(0.125, 0);
    int64_t nanosec;
};

class SubscriberTopic5
    : public rclcpp::Node
{
    ofstream& outfile_apr_;
    ofstream& outfile_seam_;
    ofstream& outfile_computetime_apr;
    ofstream& outfile_computetime_seam;
    double minus_apr;
    double minus_seam;

    public:
    SubscriberTopic5(int period1, int period2, int period3, int period4, int period5, int x_ms, int length_ms, int BOUND_Between_ms, std::ofstream& outfile_apr, std::ofstream& outfile_seam, std::ofstream& outfile_ct_apr, std::ofstream& outfile_ct_seam) :
        Node("subscriber_topic5"), outfile_apr_(outfile_apr), outfile_seam_(outfile_seam), outfile_computetime_apr(outfile_ct_apr), outfile_computetime_seam(outfile_ct_seam), 
        apr_sync_(buffer_size), seam_sync_(), 
        period1_(period1), period2_(period2), period3_(period3), period4_(period4), period5_(period5), x_ms_(x_ms), length_ms_(length_ms), Bound_between_ms_(BOUND_Between_ms)
    {
        callback_group_subscriber_ = this->create_callback_group(
        rclcpp::callback_group::CallbackGroupType::Reentrant);

        // Each of these callback groups is basically a thread
        // Everything assigned to one of them gets bundled into the same thread
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_subscriber_;
        
        topic1_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic1", 300, 
                                                                        std::bind(&SubscriberTopic5::callback1, this, _1), sub_opt);

        topic2_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic2", 300, 
                                                                        std::bind(&SubscriberTopic5::callback2, this, _1), sub_opt);

        topic3_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic3", 300, 
                                                                        std::bind(&SubscriberTopic5::callback3, this, _1), sub_opt);

        topic4_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic4", 300, 
                                                                        std::bind(&SubscriberTopic5::callback4, this, _1), sub_opt);

        topic5_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic5", 300, 
                                                                        std::bind(&SubscriberTopic5::callback5, this, _1), sub_opt);

        apr_sync_.registerCallback(std::bind(&SubscriberTopic5::apr_callback, this, _1, _2, _3, _4, _5));
        seam_sync_.registerCallback(std::bind(&SubscriberTopic5::seam_callback, this, _1, _2, _3, _4, _5));

        RCLCPP_INFO(this->get_logger(), "Period 1: %d, Period 2: %d, Period 3: %d, Period 4: %d, Period 5: %d",
                    period1, period2, period3, period4, period5);

        apr_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        apr_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        apr_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);
        apr_sync_.setInterMessageLowerBound(3, PeriodBase * 0.001 * period4);
        apr_sync_.setInterMessageLowerBound(4, PeriodBase * 0.001 * period5);

        seam_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        seam_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        seam_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);
        seam_sync_.setInterMessageLowerBound(3, PeriodBase * 0.001 * period4);
        seam_sync_.setInterMessageLowerBound(4, PeriodBase * 0.001 * period5);
        
        apr_sync_.setAgePenalty(0);
        // seam_sync_.setAgePenalty(0);
        seam_sync_.setBOUND(x_ms);

        suc_apr = 0;
        suc_seam = 0;

        nanosec = Bound_between_ms_ * 1000000;
        rclcpp::Duration duration(nanosec);
        BOUND_between_ = duration;

        stop_timer_ = this->create_wall_timer(std::chrono::milliseconds(length_ms_), std::bind(&SubscriberTopic5::stop_receive, this));
    }

    ~SubscriberTopic5()
    {
    }

    private:
    void stop_receive()
    {
        RCLCPP_INFO(this->get_logger(), "Time is up! This subscriber has stopped receiving!");
        kill(getpid(),SIGINT);
    }

    void apr_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg5)
    {
        double topic1_timestamp = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double topic2_timestamp = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        double topic3_timestamp = (double)msg3->header.stamp.sec + 1e-9*(double)msg3->header.stamp.nanosec;
        double topic4_timestamp = (double)msg4->header.stamp.sec + 1e-9*(double)msg4->header.stamp.nanosec;
        double topic5_timestamp = (double)msg5->header.stamp.sec + 1e-9*(double)msg5->header.stamp.nanosec;

        outfile_apr_ << topic1_timestamp << " " << topic2_timestamp << " " << topic3_timestamp << " " << topic4_timestamp << " " << topic5_timestamp << endl;        
        
        double time_disparity = cal_time_disparity(5, topic1_timestamp, topic2_timestamp, topic3_timestamp, topic4_timestamp, topic5_timestamp);

        count_apr++;

        if (apr_first_sync_)
        {
            apr_pre_sync_time_ = this->now();
            apr_first_sync_ = false;
        }
        else
        {
            apr_this_sync_time_ = this->now();
            if ((time_disparity < x_ms_) && ((apr_this_sync_time_ - apr_pre_sync_time_) < BOUND_between_))
            {
                suc_apr++;
            }
            apr_pre_sync_time_ = apr_this_sync_time_;
        }
    }

    void seam_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4,
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg5)
    {
        double topic1_timestamp = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double topic2_timestamp = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        double topic3_timestamp = (double)msg3->header.stamp.sec + 1e-9*(double)msg3->header.stamp.nanosec;
        double topic4_timestamp = (double)msg4->header.stamp.sec + 1e-9*(double)msg4->header.stamp.nanosec;
        double topic5_timestamp = (double)msg5->header.stamp.sec + 1e-9*(double)msg5->header.stamp.nanosec;

        outfile_seam_ << topic1_timestamp << " " << topic2_timestamp << " " << topic3_timestamp << " " << topic4_timestamp << " " << topic5_timestamp << endl;
        double time_disparity = cal_time_disparity(5, topic1_timestamp, topic2_timestamp, topic3_timestamp, topic4_timestamp, topic5_timestamp);
        
        count_seam++;

        if (seam_first_sync_)
        {
            seam_pre_sync_time_ = this->now();
            seam_first_sync_ = false;
        }
        else
        {
            seam_this_sync_time_ = this->now();
            if ((time_disparity < x_ms_) && ((seam_this_sync_time_ - seam_pre_sync_time_) < BOUND_between_))
            {
                suc_seam++;
            }
            seam_pre_sync_time_ = seam_this_sync_time_;
        }
    }

    void callback1(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic0  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<0>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic0  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic0  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<0>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic0  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback2(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic1  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<1>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic1  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic1  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<1>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic1  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback3(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic2  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<2>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic2  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic2  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<2>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic2  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback4(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic3  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<3>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic3  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic3  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<3>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic3  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback5(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {       
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic4  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<4>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic4  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic4  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<4>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic4  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_subscriber_;
    rclcpp::TimerBase::SharedPtr stop_timer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic1_sub_, topic2_sub_, topic3_sub_, topic4_sub_, topic5_sub_;

    typedef Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > AlgSync;
    typedef Synchronizer<sync_policies::SEAM<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > SeamSync;

    AlgSync apr_sync_;
    SeamSync seam_sync_;

    int period1_, period2_, period3_, period4_, period5_;
    int x_ms_;
    int length_ms_;
    rclcpp::Time apr_pre_sync_time_;
    rclcpp::Time apr_this_sync_time_;
    rclcpp::Time seam_pre_sync_time_;
    rclcpp::Time seam_this_sync_time_;
    bool apr_first_sync_ = true;
    bool seam_first_sync_ = true;

    int Bound_between_ms_;
    rclcpp::Duration BOUND_between_ = rclcpp::Duration(0.125, 0);
    int64_t nanosec;
};

class SubscriberTopic6
    : public rclcpp::Node
{
    ofstream& outfile_apr_;
    ofstream& outfile_seam_;
    ofstream& outfile_computetime_apr;
    ofstream& outfile_computetime_seam;
    double minus_apr;
    double minus_seam;

    public:
    SubscriberTopic6(int period1, int period2, int period3, int period4, int period5, int period6, int x_ms, int length_ms, int BOUND_Between_ms, std::ofstream& outfile_apr, std::ofstream& outfile_seam, std::ofstream& outfile_ct_apr, std::ofstream& outfile_ct_seam) :
        Node("subscriber_topic6"), outfile_apr_(outfile_apr), outfile_seam_(outfile_seam), outfile_computetime_apr(outfile_ct_apr), outfile_computetime_seam(outfile_ct_seam), 
        apr_sync_(buffer_size), seam_sync_(), 
        period1_(period1), period2_(period2), period3_(period3), period4_(period4), period5_(period5), period6_(period6), x_ms_(x_ms), length_ms_(length_ms), Bound_between_ms_(BOUND_Between_ms)
    {
        callback_group_subscriber_ = this->create_callback_group(
        rclcpp::callback_group::CallbackGroupType::Reentrant);

        // Each of these callback groups is basically a thread
        // Everything assigned to one of them gets bundled into the same thread
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_subscriber_;

        topic1_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic1", 300, 
                                                                        std::bind(&SubscriberTopic6::callback1, this, _1), sub_opt);

        topic2_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic2", 300, 
                                                                        std::bind(&SubscriberTopic6::callback2, this, _1), sub_opt);

        topic3_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic3", 300, 
                                                                        std::bind(&SubscriberTopic6::callback3, this, _1), sub_opt);

        topic4_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic4", 300, 
                                                                        std::bind(&SubscriberTopic6::callback4, this, _1), sub_opt);

        topic5_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic5", 300, 
                                                                        std::bind(&SubscriberTopic6::callback5, this, _1), sub_opt);

        topic6_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic6", 300, 
                                                                        std::bind(&SubscriberTopic6::callback6, this, _1), sub_opt);

        apr_sync_.registerCallback(std::bind(&SubscriberTopic6::apr_callback, this, _1, _2, _3, _4, _5, _6));
        seam_sync_.registerCallback(std::bind(&SubscriberTopic6::seam_callback, this, _1, _2, _3, _4, _5, _6));

        RCLCPP_INFO(this->get_logger(), "Period 1: %d, Period 2: %d, Period 3: %d, Period 4: %d, Period 5: %d, Period 6: %d",
                    period1, period2, period3, period4, period5, period6);

        apr_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        apr_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        apr_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);
        apr_sync_.setInterMessageLowerBound(3, PeriodBase * 0.001 * period4);
        apr_sync_.setInterMessageLowerBound(4, PeriodBase * 0.001 * period5);
        apr_sync_.setInterMessageLowerBound(5, PeriodBase * 0.001 * period6);
        
        seam_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        seam_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        seam_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);
        seam_sync_.setInterMessageLowerBound(3, PeriodBase * 0.001 * period4);
        seam_sync_.setInterMessageLowerBound(4, PeriodBase * 0.001 * period5);
        seam_sync_.setInterMessageLowerBound(5, PeriodBase * 0.001 * period6);
        
        apr_sync_.setAgePenalty(0);
        // seam_sync_.setAgePenalty(0);
        seam_sync_.setBOUND(x_ms);

        suc_apr = 0;
        suc_seam = 0;

        nanosec = Bound_between_ms_ * 1000000;
        rclcpp::Duration duration(nanosec);
        BOUND_between_ = duration;

        stop_timer_ = this->create_wall_timer(std::chrono::milliseconds(length_ms_), std::bind(&SubscriberTopic6::stop_receive, this));
    }

    ~SubscriberTopic6()
    {
    }

    private:
    void stop_receive()
    {
        RCLCPP_INFO(this->get_logger(), "Time is up! This subscriber has stopped receiving!");
        kill(getpid(),SIGINT);
    }

    void apr_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg5, const sensor_msgs::msg::JointState::ConstSharedPtr& msg6)
    {
        double topic1_timestamp = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double topic2_timestamp = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        double topic3_timestamp = (double)msg3->header.stamp.sec + 1e-9*(double)msg3->header.stamp.nanosec;
        double topic4_timestamp = (double)msg4->header.stamp.sec + 1e-9*(double)msg4->header.stamp.nanosec;
        double topic5_timestamp = (double)msg5->header.stamp.sec + 1e-9*(double)msg5->header.stamp.nanosec;
        double topic6_timestamp = (double)msg6->header.stamp.sec + 1e-9*(double)msg6->header.stamp.nanosec;

        outfile_apr_ << topic1_timestamp << " " << topic2_timestamp << " " << topic3_timestamp << " " << topic4_timestamp << " " << topic5_timestamp << " " << topic6_timestamp << endl;
        double time_disparity = cal_time_disparity(6, topic1_timestamp, topic2_timestamp, topic3_timestamp, topic4_timestamp, topic5_timestamp, topic6_timestamp);

        count_apr++;

        if (apr_first_sync_)
        {
            apr_pre_sync_time_ = this->now();
            apr_first_sync_ = false;
        }
        else
        {
            apr_this_sync_time_ = this->now();
            if ((time_disparity < x_ms_) && ((apr_this_sync_time_ - apr_pre_sync_time_) < BOUND_between_))
            {
                suc_apr++;
            }
            apr_pre_sync_time_ = apr_this_sync_time_;
        }
    }

    void seam_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4,
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg5, const sensor_msgs::msg::JointState::ConstSharedPtr& msg6)
    {
        double topic1_timestamp = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double topic2_timestamp = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        double topic3_timestamp = (double)msg3->header.stamp.sec + 1e-9*(double)msg3->header.stamp.nanosec;
        double topic4_timestamp = (double)msg4->header.stamp.sec + 1e-9*(double)msg4->header.stamp.nanosec;
        double topic5_timestamp = (double)msg5->header.stamp.sec + 1e-9*(double)msg5->header.stamp.nanosec;
        double topic6_timestamp = (double)msg6->header.stamp.sec + 1e-9*(double)msg6->header.stamp.nanosec;

        outfile_seam_ << topic1_timestamp << " " << topic2_timestamp << " " << topic3_timestamp << " " << topic4_timestamp << " " << topic5_timestamp << " " << topic6_timestamp << endl;
        double time_disparity = cal_time_disparity(6, topic1_timestamp, topic2_timestamp, topic3_timestamp, topic4_timestamp, topic5_timestamp, topic6_timestamp);
        
        count_seam++;

        if (seam_first_sync_)
        {
            seam_pre_sync_time_ = this->now();
            seam_first_sync_ = false;
        }
        else
        {
            seam_this_sync_time_ = this->now();
            if ((time_disparity < x_ms_) && ((seam_this_sync_time_ - seam_pre_sync_time_) < BOUND_between_))
            {
                suc_seam++;
            }
            seam_pre_sync_time_ = seam_this_sync_time_;
        }
    }

    void callback1(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {       
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic0  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<0>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic0  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic0  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<0>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic0  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback2(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic1  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<1>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic1  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic1  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<1>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic1  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback3(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic2  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<2>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic2  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic2  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<2>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic2  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback4(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic3  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<3>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic3  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic3  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<3>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic3  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback5(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {       
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic4  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<4>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic4  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic4  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<4>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic4  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback6(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic5  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<5>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic5  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic5  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<5>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic5  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_subscriber_;
    rclcpp::TimerBase::SharedPtr stop_timer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic1_sub_, topic2_sub_, topic3_sub_, topic4_sub_, topic5_sub_, topic6_sub_;

    typedef Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > AlgSync;
    typedef Synchronizer<sync_policies::SEAM<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > SeamSync;

    AlgSync apr_sync_;
    SeamSync seam_sync_;

    int period1_, period2_, period3_, period4_, period5_, period6_;
    int x_ms_;
    int length_ms_;
    rclcpp::Time apr_pre_sync_time_;
    rclcpp::Time apr_this_sync_time_;
    rclcpp::Time seam_pre_sync_time_;
    rclcpp::Time seam_this_sync_time_;
    bool apr_first_sync_ = true;
    bool seam_first_sync_ = true;
    
    int Bound_between_ms_;
    rclcpp::Duration BOUND_between_ = rclcpp::Duration(0.125, 0);
    int64_t nanosec;
};

class SubscriberTopic7
    : public rclcpp::Node
{
    ofstream& outfile_apr_;
    ofstream& outfile_seam_;
    ofstream& outfile_computetime_apr;
    ofstream& outfile_computetime_seam;
    double minus_apr;
    double minus_seam;

    public:
    SubscriberTopic7(int period1, int period2, int period3, int period4, int period5, int period6, int period7, int x_ms, int length_ms, int BOUND_Between_ms, std::ofstream& outfile_apr, std::ofstream& outfile_seam, std::ofstream& outfile_ct_apr, std::ofstream& outfile_ct_seam) :
        Node("subscriber_topic7"), outfile_apr_(outfile_apr), outfile_seam_(outfile_seam), outfile_computetime_apr(outfile_ct_apr), outfile_computetime_seam(outfile_ct_seam), 
        apr_sync_(buffer_size), seam_sync_(), 
        period1_(period1), period2_(period2), period3_(period3), period4_(period4), period5_(period5), period6_(period6), period7_(period7), x_ms_(x_ms), length_ms_(length_ms), Bound_between_ms_(BOUND_Between_ms)
    {
        callback_group_subscriber_ = this->create_callback_group(
        rclcpp::callback_group::CallbackGroupType::Reentrant);

        // Each of these callback groups is basically a thread
        // Everything assigned to one of them gets bundled into the same thread
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_subscriber_;

        topic1_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic1", 300, 
                                                                        std::bind(&SubscriberTopic7::callback1, this, _1), sub_opt);

        topic2_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic2", 300, 
                                                                        std::bind(&SubscriberTopic7::callback2, this, _1), sub_opt);

        topic3_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic3", 300, 
                                                                        std::bind(&SubscriberTopic7::callback3, this, _1), sub_opt);

        topic4_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic4", 300, 
                                                                        std::bind(&SubscriberTopic7::callback4, this, _1), sub_opt);

        topic5_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic5", 300, 
                                                                        std::bind(&SubscriberTopic7::callback5, this, _1), sub_opt);

        topic6_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic6", 300, 
                                                                        std::bind(&SubscriberTopic7::callback6, this, _1), sub_opt);

        topic7_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic7", 300, 
                                                                        std::bind(&SubscriberTopic7::callback7, this, _1), sub_opt);

        apr_sync_.registerCallback(std::bind(&SubscriberTopic7::apr_callback, this, _1, _2, _3, _4, _5, _6, _7));
        seam_sync_.registerCallback(std::bind(&SubscriberTopic7::seam_callback, this, _1, _2, _3, _4, _5, _6, _7));

        RCLCPP_INFO(this->get_logger(), "Period 1: %d, Period 2: %d, Period 3: %d, Period 4: %d, Period 5: %d, Period 6: %d, Period 7: %d",
                    period1, period2, period3, period4, period5, period6, period7);

        apr_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        apr_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        apr_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);
        apr_sync_.setInterMessageLowerBound(3, PeriodBase * 0.001 * period4);
        apr_sync_.setInterMessageLowerBound(4, PeriodBase * 0.001 * period5);
        apr_sync_.setInterMessageLowerBound(5, PeriodBase * 0.001 * period6);
        apr_sync_.setInterMessageLowerBound(6, PeriodBase * 0.001 * period7);

        seam_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        seam_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        seam_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);
        seam_sync_.setInterMessageLowerBound(3, PeriodBase * 0.001 * period4);
        seam_sync_.setInterMessageLowerBound(4, PeriodBase * 0.001 * period5);
        seam_sync_.setInterMessageLowerBound(5, PeriodBase * 0.001 * period6);
        seam_sync_.setInterMessageLowerBound(6, PeriodBase * 0.001 * period7);
        
        apr_sync_.setAgePenalty(0);
        // seam_sync_.setAgePenalty(0);
        seam_sync_.setBOUND(x_ms);

        suc_apr = 0;
        suc_seam = 0;

        nanosec = Bound_between_ms_ * 1000000;
        rclcpp::Duration duration(nanosec);
        BOUND_between_ = duration;

        stop_timer_ = this->create_wall_timer(std::chrono::milliseconds(length_ms_), std::bind(&SubscriberTopic7::stop_receive, this));
    }

    ~SubscriberTopic7()
    {
    }

    private:
    void stop_receive()
    {
        RCLCPP_INFO(this->get_logger(), "Time is up! This subscriber has stopped receiving!");
        kill(getpid(),SIGINT);
    }

    void apr_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg5, const sensor_msgs::msg::JointState::ConstSharedPtr& msg6, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg7)
    {
        double topic1_timestamp = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double topic2_timestamp = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        double topic3_timestamp = (double)msg3->header.stamp.sec + 1e-9*(double)msg3->header.stamp.nanosec;
        double topic4_timestamp = (double)msg4->header.stamp.sec + 1e-9*(double)msg4->header.stamp.nanosec;
        double topic5_timestamp = (double)msg5->header.stamp.sec + 1e-9*(double)msg5->header.stamp.nanosec;
        double topic6_timestamp = (double)msg6->header.stamp.sec + 1e-9*(double)msg6->header.stamp.nanosec;
        double topic7_timestamp = (double)msg7->header.stamp.sec + 1e-9*(double)msg7->header.stamp.nanosec;

        outfile_apr_ << topic1_timestamp << " " << topic2_timestamp << " " << topic3_timestamp << " " << topic4_timestamp << " " << topic5_timestamp << " " << topic6_timestamp << " " << topic7_timestamp << endl;
        double time_disparity = cal_time_disparity(7, topic1_timestamp, topic2_timestamp, topic3_timestamp, topic4_timestamp, topic5_timestamp, topic6_timestamp, topic7_timestamp);
        
        count_apr++;

        if (apr_first_sync_)
        {
            apr_pre_sync_time_ = this->now();
            apr_first_sync_ = false;
        }
        else
        {
            apr_this_sync_time_ = this->now();
            if ((time_disparity < x_ms_) && ((apr_this_sync_time_ - apr_pre_sync_time_) < BOUND_between_))
            {
                suc_apr++;
            }
            apr_pre_sync_time_ = apr_this_sync_time_;
        }
    }

    void seam_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4,
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg5, const sensor_msgs::msg::JointState::ConstSharedPtr& msg6, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg7)
    {
        double topic1_timestamp = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double topic2_timestamp = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        double topic3_timestamp = (double)msg3->header.stamp.sec + 1e-9*(double)msg3->header.stamp.nanosec;
        double topic4_timestamp = (double)msg4->header.stamp.sec + 1e-9*(double)msg4->header.stamp.nanosec;
        double topic5_timestamp = (double)msg5->header.stamp.sec + 1e-9*(double)msg5->header.stamp.nanosec;
        double topic6_timestamp = (double)msg6->header.stamp.sec + 1e-9*(double)msg6->header.stamp.nanosec;
        double topic7_timestamp = (double)msg7->header.stamp.sec + 1e-9*(double)msg7->header.stamp.nanosec;

        outfile_seam_ << topic1_timestamp << " " << topic2_timestamp << " " << topic3_timestamp << " " << topic4_timestamp << " " << topic5_timestamp << " " << topic6_timestamp << " " << topic7_timestamp << endl;
        double time_disparity = cal_time_disparity(7, topic1_timestamp, topic2_timestamp, topic3_timestamp, topic4_timestamp, topic5_timestamp, topic6_timestamp, topic7_timestamp);
        
        count_seam++;

        if (seam_first_sync_)
        {
            seam_pre_sync_time_ = this->now();
            seam_first_sync_ = false;
        }
        else
        {
            seam_this_sync_time_ = this->now();
            if ((time_disparity < x_ms_) && ((seam_this_sync_time_ - seam_pre_sync_time_) < BOUND_between_))
            {
                suc_seam++;
            }
            seam_pre_sync_time_ = seam_this_sync_time_;
        }
    }

    void callback1(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {       
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic0  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<0>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic0  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic0  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<0>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic0  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback2(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic1  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<1>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic1  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic1  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<1>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic1  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback3(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic2  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<2>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic2  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic2  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<2>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic2  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback4(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic3  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<3>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic3  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic3  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<3>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic3  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback5(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {       
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic4  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<4>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic4  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic4  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<4>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic4  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback6(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic5  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<5>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic5  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic5  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<5>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic5  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback7(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic6  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<6>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic6  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic6  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<6>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic6  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_subscriber_;
    rclcpp::TimerBase::SharedPtr stop_timer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic1_sub_, topic2_sub_, topic3_sub_, topic4_sub_, topic5_sub_, topic6_sub_, topic7_sub_;

    typedef Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > AlgSync;
    typedef Synchronizer<sync_policies::SEAM<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > SeamSync;

    AlgSync apr_sync_;
    SeamSync seam_sync_;

    int period1_, period2_, period3_, period4_, period5_, period6_, period7_;
    int x_ms_;
    int length_ms_;
    rclcpp::Time apr_pre_sync_time_;
    rclcpp::Time apr_this_sync_time_;
    rclcpp::Time seam_pre_sync_time_;
    rclcpp::Time seam_this_sync_time_;
    bool apr_first_sync_ = true;
    bool seam_first_sync_ = true;

    int Bound_between_ms_;
    rclcpp::Duration BOUND_between_ = rclcpp::Duration(0.125, 0);
    int64_t nanosec;
};

class SubscriberTopic8
    : public rclcpp::Node
{
    ofstream& outfile_apr_;
    ofstream& outfile_seam_;
    ofstream& outfile_computetime_apr;
    ofstream& outfile_computetime_seam;
    double minus_apr;
    double minus_seam;

    public:
    SubscriberTopic8(int period1, int period2, int period3, int period4, int period5, int period6, int period7, int period8, int x_ms, int length_ms, int BOUND_Between_ms, std::ofstream& outfile_apr, std::ofstream& outfile_seam, std::ofstream& outfile_ct_apr, std::ofstream& outfile_ct_seam) :
        Node("subscriber_topic8"), outfile_apr_(outfile_apr), outfile_seam_(outfile_seam), outfile_computetime_apr(outfile_ct_apr), outfile_computetime_seam(outfile_ct_seam), 
        apr_sync_(buffer_size), seam_sync_(), 
        period1_(period1), period2_(period2), period3_(period3), period4_(period4), period5_(period5), period6_(period6), period7_(period7), period8_(period8), x_ms_(x_ms), length_ms_(length_ms), Bound_between_ms_(BOUND_Between_ms)
    {
        callback_group_subscriber_ = this->create_callback_group(
        rclcpp::callback_group::CallbackGroupType::Reentrant);

        // Each of these callback groups is basically a thread
        // Everything assigned to one of them gets bundled into the same thread
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_subscriber_;

        topic1_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic1", 300, 
                                                                        std::bind(&SubscriberTopic8::callback1, this, _1), sub_opt);

        topic2_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic2", 300, 
                                                                        std::bind(&SubscriberTopic8::callback2, this, _1), sub_opt);

        topic3_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic3", 300, 
                                                                        std::bind(&SubscriberTopic8::callback3, this, _1), sub_opt);

        topic4_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic4", 300, 
                                                                        std::bind(&SubscriberTopic8::callback4, this, _1), sub_opt);

        topic5_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic5", 300, 
                                                                        std::bind(&SubscriberTopic8::callback5, this, _1), sub_opt);

        topic6_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic6", 300, 
                                                                        std::bind(&SubscriberTopic8::callback6, this, _1), sub_opt);

        topic7_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic7", 300, 
                                                                        std::bind(&SubscriberTopic8::callback7, this, _1), sub_opt);

        topic8_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic8", 300, 
                                                                        std::bind(&SubscriberTopic8::callback8, this, _1), sub_opt);

        apr_sync_.registerCallback(std::bind(&SubscriberTopic8::apr_callback, this, _1, _2, _3, _4, _5, _6, _7, _8));
        seam_sync_.registerCallback(std::bind(&SubscriberTopic8::seam_callback, this, _1, _2, _3, _4, _5, _6, _7, _8));

        RCLCPP_INFO(this->get_logger(), "Period 1: %d, Period 2: %d, Period 3: %d, Period 4: %d, Period 5: %d, Period 6: %d, Period 7: %d, Period 8: %d",
                    period1, period2, period3, period4, period5, period6, period7, period8);

        apr_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        apr_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        apr_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);
        apr_sync_.setInterMessageLowerBound(3, PeriodBase * 0.001 * period4);
        apr_sync_.setInterMessageLowerBound(4, PeriodBase * 0.001 * period5);
        apr_sync_.setInterMessageLowerBound(5, PeriodBase * 0.001 * period6);
        apr_sync_.setInterMessageLowerBound(6, PeriodBase * 0.001 * period7);
        apr_sync_.setInterMessageLowerBound(7, PeriodBase * 0.001 * period8);

        seam_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        seam_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        seam_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);
        seam_sync_.setInterMessageLowerBound(3, PeriodBase * 0.001 * period4);
        seam_sync_.setInterMessageLowerBound(4, PeriodBase * 0.001 * period5);
        seam_sync_.setInterMessageLowerBound(5, PeriodBase * 0.001 * period6);
        seam_sync_.setInterMessageLowerBound(6, PeriodBase * 0.001 * period7);
        seam_sync_.setInterMessageLowerBound(7, PeriodBase * 0.001 * period8);
        
        apr_sync_.setAgePenalty(0);
        // seam_sync_.setAgePenalty(0);
        seam_sync_.setBOUND(x_ms);

        suc_apr = 0;
        suc_seam = 0;

        nanosec = Bound_between_ms_ * 1000000;
        rclcpp::Duration duration(nanosec);
        BOUND_between_ = duration;

        stop_timer_ = this->create_wall_timer(std::chrono::milliseconds(length_ms_), std::bind(&SubscriberTopic8::stop_receive, this));
    }

    ~SubscriberTopic8()
    {
    }

    private:
    void stop_receive()
    {
        RCLCPP_INFO(this->get_logger(), "Time is up! This subscriber has stopped receiving!");
        kill(getpid(),SIGINT);
    }

    void apr_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg5, const sensor_msgs::msg::JointState::ConstSharedPtr& msg6, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg7, const sensor_msgs::msg::JointState::ConstSharedPtr& msg8)
    {
        double topic1_timestamp = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double topic2_timestamp = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        double topic3_timestamp = (double)msg3->header.stamp.sec + 1e-9*(double)msg3->header.stamp.nanosec;
        double topic4_timestamp = (double)msg4->header.stamp.sec + 1e-9*(double)msg4->header.stamp.nanosec;
        double topic5_timestamp = (double)msg5->header.stamp.sec + 1e-9*(double)msg5->header.stamp.nanosec;
        double topic6_timestamp = (double)msg6->header.stamp.sec + 1e-9*(double)msg6->header.stamp.nanosec;
        double topic7_timestamp = (double)msg7->header.stamp.sec + 1e-9*(double)msg7->header.stamp.nanosec;
        double topic8_timestamp = (double)msg8->header.stamp.sec + 1e-9*(double)msg8->header.stamp.nanosec;

        outfile_apr_ << topic1_timestamp << " " << topic2_timestamp << " " << topic3_timestamp << " " << topic4_timestamp << " " << topic5_timestamp << " " << topic6_timestamp << " " << topic7_timestamp << " " << topic8_timestamp << endl;
        double time_disparity = cal_time_disparity(8, topic1_timestamp, topic2_timestamp, topic3_timestamp, topic4_timestamp, topic5_timestamp, topic6_timestamp, topic7_timestamp, topic8_timestamp);
        
        count_apr++;

        if (apr_first_sync_)
        {
            apr_pre_sync_time_ = this->now();
            apr_first_sync_ = false;
        }
        else
        {
            apr_this_sync_time_ = this->now();
            if ((time_disparity < x_ms_) && ((apr_this_sync_time_ - apr_pre_sync_time_) < BOUND_between_))
            {
                suc_apr++;
            }
            apr_pre_sync_time_ = apr_this_sync_time_;
        }
    }

    void seam_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4,
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg5, const sensor_msgs::msg::JointState::ConstSharedPtr& msg6, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg7, const sensor_msgs::msg::JointState::ConstSharedPtr& msg8)
    {
        double topic1_timestamp = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double topic2_timestamp = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        double topic3_timestamp = (double)msg3->header.stamp.sec + 1e-9*(double)msg3->header.stamp.nanosec;
        double topic4_timestamp = (double)msg4->header.stamp.sec + 1e-9*(double)msg4->header.stamp.nanosec;
        double topic5_timestamp = (double)msg5->header.stamp.sec + 1e-9*(double)msg5->header.stamp.nanosec;
        double topic6_timestamp = (double)msg6->header.stamp.sec + 1e-9*(double)msg6->header.stamp.nanosec;
        double topic7_timestamp = (double)msg7->header.stamp.sec + 1e-9*(double)msg7->header.stamp.nanosec;
        double topic8_timestamp = (double)msg8->header.stamp.sec + 1e-9*(double)msg8->header.stamp.nanosec;

        outfile_seam_ << topic1_timestamp << " " << topic2_timestamp << " " << topic3_timestamp << " " << topic4_timestamp << " " << topic5_timestamp << " " << topic6_timestamp << " " << topic7_timestamp << " " << topic8_timestamp << endl;
        double time_disparity = cal_time_disparity(8, topic1_timestamp, topic2_timestamp, topic3_timestamp, topic4_timestamp, topic5_timestamp, topic6_timestamp, topic7_timestamp, topic8_timestamp);
        
        count_seam++;

        if (seam_first_sync_)
        {
            seam_pre_sync_time_ = this->now();
            seam_first_sync_ = false;
        }
        else
        {
            seam_this_sync_time_ = this->now();
            if ((time_disparity < x_ms_) && ((seam_this_sync_time_ - seam_pre_sync_time_) < BOUND_between_))
            {
                suc_seam++;
            }
            seam_pre_sync_time_ = seam_this_sync_time_;
        }
    }

    void callback1(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {       
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic0  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<0>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic0  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic0  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<0>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic0  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback2(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic1  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<1>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic1  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic1  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<1>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic1  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback3(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic2  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<2>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic2  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic2  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<2>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic2  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback4(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic3  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<3>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic3  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic3  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<3>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic3  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback5(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {    
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic4  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<4>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic4  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic4  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<4>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic4  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;  
    }

    void callback6(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic5  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<5>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic5  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic5  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<5>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic5  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;     
    }

    void callback7(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic6  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<6>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic6  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic6  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<6>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic6  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback8(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic7  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<7>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic7  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic7  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<7>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic7  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;     
    }

    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_subscriber_;
    rclcpp::TimerBase::SharedPtr stop_timer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic1_sub_, topic2_sub_, topic3_sub_, topic4_sub_, topic5_sub_, topic6_sub_, topic7_sub_, topic8_sub_;

    typedef Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > AlgSync;
    typedef Synchronizer<sync_policies::SEAM<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > SeamSync;

    AlgSync apr_sync_;
    SeamSync seam_sync_;

    int period1_, period2_, period3_, period4_, period5_, period6_, period7_, period8_;
    int x_ms_;
    int length_ms_;
    rclcpp::Time apr_pre_sync_time_;
    rclcpp::Time apr_this_sync_time_;
    rclcpp::Time seam_pre_sync_time_;
    rclcpp::Time seam_this_sync_time_;
    bool apr_first_sync_ = true;
    bool seam_first_sync_ = true;

    int Bound_between_ms_;
    rclcpp::Duration BOUND_between_ = rclcpp::Duration(0.125, 0);
    int64_t nanosec;
};

class SubscriberTopic9
    : public rclcpp::Node
{
    ofstream& outfile_apr_;
    ofstream& outfile_seam_;
    ofstream& outfile_computetime_apr;
    ofstream& outfile_computetime_seam;
    double minus_apr;
    double minus_seam;

    public:
    SubscriberTopic9(int period1, int period2, int period3, int period4, int period5, int period6, int period7, int period8, int period9, int x_ms, int length_ms, int BOUND_Between_ms, std::ofstream& outfile_apr, std::ofstream& outfile_seam, std::ofstream& outfile_ct_apr, std::ofstream& outfile_ct_seam) :
        Node("subscriber_topic9"), outfile_apr_(outfile_apr), outfile_seam_(outfile_seam), outfile_computetime_apr(outfile_ct_apr), outfile_computetime_seam(outfile_ct_seam), 
        apr_sync_(buffer_size), seam_sync_(), 
        period1_(period1), period2_(period2), period3_(period3), period4_(period4), period5_(period5), period6_(period6), period7_(period7), period8_(period8), period9_(period9), x_ms_(x_ms), length_ms_(length_ms), Bound_between_ms_(BOUND_Between_ms)
    {
        callback_group_subscriber_ = this->create_callback_group(
        rclcpp::callback_group::CallbackGroupType::Reentrant);

        // Each of these callback groups is basically a thread
        // Everything assigned to one of them gets bundled into the same thread
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_subscriber_;

        topic1_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic1", 300, 
                                                                        std::bind(&SubscriberTopic9::callback1, this, _1), sub_opt);

        topic2_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic2", 300, 
                                                                        std::bind(&SubscriberTopic9::callback2, this, _1), sub_opt);

        topic3_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic3", 300, 
                                                                        std::bind(&SubscriberTopic9::callback3, this, _1), sub_opt);

        topic4_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic4", 300, 
                                                                        std::bind(&SubscriberTopic9::callback4, this, _1), sub_opt);

        topic5_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic5", 300, 
                                                                        std::bind(&SubscriberTopic9::callback5, this, _1), sub_opt);

        topic6_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic6", 300, 
                                                                        std::bind(&SubscriberTopic9::callback6, this, _1), sub_opt);

        topic7_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic7", 300, 
                                                                        std::bind(&SubscriberTopic9::callback7, this, _1), sub_opt);

        topic8_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic8", 300, 
                                                                        std::bind(&SubscriberTopic9::callback8, this, _1), sub_opt);

        topic9_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic8", 300, 
                                                                        std::bind(&SubscriberTopic9::callback9, this, _1), sub_opt);

        apr_sync_.registerCallback(std::bind(&SubscriberTopic9::apr_callback, this, _1, _2, _3, _4, _5, _6, _7, _8, _9));
        seam_sync_.registerCallback(std::bind(&SubscriberTopic9::seam_callback, this, _1, _2, _3, _4, _5, _6, _7, _8, _9));

        RCLCPP_INFO(this->get_logger(), "Period 1: %d, Period 2: %d, Period 3: %d, Period 4: %d, Period 5: %d, Period 6: %d, Period 7: %d, Period 8: %d, Period 9: %d",
                    period1, period2, period3, period4, period5, period6, period7, period8, period9);

        apr_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        apr_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        apr_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);
        apr_sync_.setInterMessageLowerBound(3, PeriodBase * 0.001 * period4);
        apr_sync_.setInterMessageLowerBound(4, PeriodBase * 0.001 * period5);
        apr_sync_.setInterMessageLowerBound(5, PeriodBase * 0.001 * period6);
        apr_sync_.setInterMessageLowerBound(6, PeriodBase * 0.001 * period7);
        apr_sync_.setInterMessageLowerBound(7, PeriodBase * 0.001 * period8);
        apr_sync_.setInterMessageLowerBound(8, PeriodBase * 0.001 * period9);

        seam_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        seam_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        seam_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);
        seam_sync_.setInterMessageLowerBound(3, PeriodBase * 0.001 * period4);
        seam_sync_.setInterMessageLowerBound(4, PeriodBase * 0.001 * period5);
        seam_sync_.setInterMessageLowerBound(5, PeriodBase * 0.001 * period6);
        seam_sync_.setInterMessageLowerBound(6, PeriodBase * 0.001 * period7);
        seam_sync_.setInterMessageLowerBound(7, PeriodBase * 0.001 * period8);
        seam_sync_.setInterMessageLowerBound(8, PeriodBase * 0.001 * period9);
        
        apr_sync_.setAgePenalty(0);
        // seam_sync_.setAgePenalty(0);
        seam_sync_.setBOUND(x_ms);

        suc_apr = 0;
        suc_seam = 0;

        nanosec = Bound_between_ms_ * 1000000;
        rclcpp::Duration duration(nanosec);
        BOUND_between_ = duration;

        stop_timer_ = this->create_wall_timer(std::chrono::milliseconds(length_ms_), std::bind(&SubscriberTopic9::stop_receive, this));
    }

    ~SubscriberTopic9()
    {
    }

    private:
    void stop_receive()
    {
        RCLCPP_INFO(this->get_logger(), "Time is up! This subscriber has stopped receiving!");
        kill(getpid(),SIGINT);
    }

    void apr_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg5, const sensor_msgs::msg::JointState::ConstSharedPtr& msg6, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg7, const sensor_msgs::msg::JointState::ConstSharedPtr& msg8, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg9)
    {
        double topic1_timestamp = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double topic2_timestamp = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        double topic3_timestamp = (double)msg3->header.stamp.sec + 1e-9*(double)msg3->header.stamp.nanosec;
        double topic4_timestamp = (double)msg4->header.stamp.sec + 1e-9*(double)msg4->header.stamp.nanosec;
        double topic5_timestamp = (double)msg5->header.stamp.sec + 1e-9*(double)msg5->header.stamp.nanosec;
        double topic6_timestamp = (double)msg6->header.stamp.sec + 1e-9*(double)msg6->header.stamp.nanosec;
        double topic7_timestamp = (double)msg7->header.stamp.sec + 1e-9*(double)msg7->header.stamp.nanosec;
        double topic8_timestamp = (double)msg8->header.stamp.sec + 1e-9*(double)msg8->header.stamp.nanosec;
        double topic9_timestamp = (double)msg9->header.stamp.sec + 1e-9*(double)msg9->header.stamp.nanosec;

        outfile_apr_ << topic1_timestamp << " " << topic2_timestamp << " " << topic3_timestamp << " " << topic4_timestamp << " " << topic5_timestamp << " " << topic6_timestamp << " " << topic7_timestamp << " " << topic8_timestamp << " " << topic9_timestamp << endl;
        double time_disparity = cal_time_disparity(9, topic1_timestamp, topic2_timestamp, topic3_timestamp, topic4_timestamp, topic5_timestamp, topic6_timestamp, topic7_timestamp, topic8_timestamp, topic9_timestamp);
        
        count_apr++;

        if (apr_first_sync_)
        {
            apr_pre_sync_time_ = this->now();
            apr_first_sync_ = false;
        }
        else
        {
            apr_this_sync_time_ = this->now();
            if ((time_disparity < x_ms_) && ((apr_this_sync_time_ - apr_pre_sync_time_) < BOUND_between_))
            {
                suc_apr++;
            }
            apr_pre_sync_time_ = apr_this_sync_time_;
        }
    }

    void seam_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4,
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg5, const sensor_msgs::msg::JointState::ConstSharedPtr& msg6, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg7, const sensor_msgs::msg::JointState::ConstSharedPtr& msg8, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg9)
    {
        double topic1_timestamp = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double topic2_timestamp = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        double topic3_timestamp = (double)msg3->header.stamp.sec + 1e-9*(double)msg3->header.stamp.nanosec;
        double topic4_timestamp = (double)msg4->header.stamp.sec + 1e-9*(double)msg4->header.stamp.nanosec;
        double topic5_timestamp = (double)msg5->header.stamp.sec + 1e-9*(double)msg5->header.stamp.nanosec;
        double topic6_timestamp = (double)msg6->header.stamp.sec + 1e-9*(double)msg6->header.stamp.nanosec;
        double topic7_timestamp = (double)msg7->header.stamp.sec + 1e-9*(double)msg7->header.stamp.nanosec;
        double topic8_timestamp = (double)msg8->header.stamp.sec + 1e-9*(double)msg8->header.stamp.nanosec;
        double topic9_timestamp = (double)msg9->header.stamp.sec + 1e-9*(double)msg9->header.stamp.nanosec;
        
        outfile_seam_ << topic1_timestamp << " " << topic2_timestamp << " " << topic3_timestamp << " " << topic4_timestamp << " " << topic5_timestamp << " " << topic6_timestamp << " " << topic7_timestamp << " " << topic8_timestamp << " " << topic9_timestamp << endl;
        double time_disparity = cal_time_disparity(9, topic1_timestamp, topic2_timestamp, topic3_timestamp, topic4_timestamp, topic5_timestamp, topic6_timestamp, topic7_timestamp, topic8_timestamp, topic9_timestamp);
        
        count_seam++;

        if (seam_first_sync_)
        {
            seam_pre_sync_time_ = this->now();
            seam_first_sync_ = false;
        }
        else
        {
            seam_this_sync_time_ = this->now();
            if ((time_disparity < x_ms_) && ((seam_this_sync_time_ - seam_pre_sync_time_) < BOUND_between_))
            {
                suc_seam++;
            }
            seam_pre_sync_time_ = seam_this_sync_time_;
        }
    }

    void callback1(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    { 
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic0  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<0>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic0  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic0  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<0>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic0  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback2(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic1  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<1>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic1  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic1  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<1>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic1  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback3(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic2  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<2>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic2  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic2  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<2>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic2  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback4(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic3  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<3>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic3  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic3  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<3>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic3  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback5(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {      
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic4  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<4>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic4  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic4  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<4>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic4  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback6(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic5  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<5>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic5  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic5  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<5>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic5  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback7(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic6  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<6>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic6  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic6  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<6>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic6  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback8(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic7  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<7>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic7  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic7  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<7>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic7  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    void callback9(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        rclcpp::Time now_1 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before  add a message in topic8  by Approximate Algorithm: %.9f", 1e-9 * now_1.nanoseconds());
        apr_sync_.add<8>(msg);
        rclcpp::Time now_2 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic8  by Approximate Algorithm: %.9f", 1e-9 * now_2.nanoseconds());
        minus_apr = now_2.seconds() - now_1.seconds();
        outfile_computetime_apr << minus_apr << endl;

        rclcpp::Time now_3 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time before add a message in topic8  by SEAM Algorithm: %.9f", 1e-9 * now_3.nanoseconds());
        seam_sync_.add<8>(msg);
        rclcpp::Time now_4 = this->now();
        RCLCPP_INFO(this->get_logger(), "Time after add a message in topic8  by SEAM Algorithm: %.9f", 1e-9 * now_4.nanoseconds());
        minus_seam = now_4.seconds() - now_3.seconds();
        outfile_computetime_seam << minus_seam << endl;
    }

    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_subscriber_;
    rclcpp::TimerBase::SharedPtr stop_timer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic1_sub_, topic2_sub_, topic3_sub_, topic4_sub_, topic5_sub_, topic6_sub_, topic7_sub_, topic8_sub_, topic9_sub_;

    typedef Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > AlgSync;
    typedef Synchronizer<sync_policies::SEAM<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > SeamSync;

    AlgSync apr_sync_;
    SeamSync seam_sync_;

    int period1_, period2_, period3_, period4_, period5_, period6_, period7_, period8_, period9_;
    int x_ms_;
    int length_ms_;
    rclcpp::Time apr_pre_sync_time_;
    rclcpp::Time apr_this_sync_time_;
    rclcpp::Time seam_pre_sync_time_;
    rclcpp::Time seam_this_sync_time_;
    bool apr_first_sync_ = true;
    bool seam_first_sync_ = true;

    int Bound_between_ms_;
    rclcpp::Duration BOUND_between_ = rclcpp::Duration(0.125, 0);
    int64_t nanosec;
};
}
