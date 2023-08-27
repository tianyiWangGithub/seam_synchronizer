#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <string> 

#include "publisher.h"
#include "subscriber_seam.h"

using namespace synchronizer;
using namespace std;

extern int count_apr;
extern int count_seam;
extern int suc_apr;
extern int suc_seam;

int main(int argc, char * argv[])
{
    string topic_name;
    double success_rate_apr;
    double success_rate_seam;
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    default_random_engine gen(seed);

    ////////////////////////////////////////////////////////////////////////////////////////
    int channel_num = atoi(argv[1]);
    int lower_limit = atoi(argv[2]);
    int test_num = atoi(argv[3]);
    string result_path = argv[4];
    int BOUND_ms = atoi(argv[5]);
    int length_s = atoi(argv[6]);
    int BOUND_Between_ms = atoi(argv[7]);

    int length_ms = length_s * 1000;
    
    uniform_int_distribution<unsigned> perd(lower_limit, 100);
    ////////////////////////////////////////////////////////////////////////////////////////
    ofstream outfile_apr_;
    ofstream outfile_seam_;
    ofstream outfile_sr_;
    
    // timestamp of ApproximateTime Policy
    outfile_apr_.open(result_path + "/timestamp_apr.txt", ios::out);
    if (!outfile_apr_.is_open()) 
    {
        cout<<"Error opening file timestamp_apr.txt! "<<endl;
    }

    outfile_apr_.setf(ios::fixed, ios::floatfield);
    outfile_apr_.precision(9);

    // timestamps of SEAM Policy
    outfile_seam_.open(result_path + "/timestamp_seam.txt", ios::out);
    if (!outfile_seam_.is_open()) 
    {
        cout<<"Error opening file timestamp_seam.txt! "<<endl;
    }

    outfile_seam_.setf(ios::fixed, ios::floatfield);
    outfile_seam_.precision(9);

    // success rate of ApproximateTime and SEAM
    outfile_sr_.open(result_path + "/success_rate.txt", ios::out);
    if (!outfile_sr_.is_open()) 
    {
        cout<<"Error opening file success_rate.txt! "<<endl;
    }

    outfile_sr_.setf(ios::fixed, ios::floatfield);
    outfile_sr_.precision(9);

    switch(channel_num)
    {
    case 2:
    {
        int count = 0;
        while(count < test_num)
        {
            rclcpp::init(argc, argv);
            // rclcpp::executors::SingleThreadedExecutor executor;
            rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(),50);
            
            int real_period1 = perd(gen);
            topic_name = "topic" + to_string(1);
            auto pub_node1 = std::make_shared<Publisher>(topic_name, real_period1);
            executor.add_node(pub_node1);

            int real_period2 = perd(gen);
            topic_name = "topic" + to_string(2);
            auto pub_node2 = std::make_shared<Publisher>(topic_name, real_period2);
            executor.add_node(pub_node2);

            auto sub_node = std::make_shared<SubscriberTopic2>(real_period1, real_period2, BOUND_ms, length_ms, BOUND_Between_ms, outfile_apr_, outfile_seam_);
            executor.add_node(sub_node);

            executor.spin();

            executor.remove_node(pub_node1);
            executor.remove_node(pub_node2);
            executor.remove_node(sub_node);

            cout << " We have finished test " << count + 1 << endl;
            count++;
        }
        break;
    }
    case 3:
    {   
        int count = 0;
        while(count < test_num)
        {
            rclcpp::init(argc, argv);
            // rclcpp::executors::SingleThreadedExecutor executor;
            rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(),50);

            int real_period1 = perd(gen);
            topic_name = "topic" + to_string(1);
            auto pub_node1 = std::make_shared<Publisher>(topic_name, real_period1);
            executor.add_node(pub_node1);

            int real_period2 = perd(gen);
            topic_name = "topic" + to_string(2);
            auto pub_node2 = std::make_shared<Publisher>(topic_name, real_period2);
            executor.add_node(pub_node2);

            int real_period3 = perd(gen);
            topic_name = "topic" + to_string(3);
            auto pub_node3 = std::make_shared<Publisher>(topic_name, real_period3);
            executor.add_node(pub_node3);

            auto sub_node = std::make_shared<SubscriberTopic3>(real_period1, real_period2, real_period3, BOUND_ms, length_ms, BOUND_Between_ms, outfile_apr_, outfile_seam_);
            executor.add_node(sub_node);

            executor.spin();
            
            executor.remove_node(pub_node1);
            executor.remove_node(pub_node2);
            executor.remove_node(pub_node3);
            executor.remove_node(sub_node);

            cout << " We have finished test " << count + 1 << endl;
            count++;
        }

        break;
    }
    case 4:
    {
        int count = 0;
        while(count < test_num)
        {
            rclcpp::init(argc, argv);
            // rclcpp::executors::SingleThreadedExecutor executor;
            rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(),50);

            int real_period1 = perd(gen);
            topic_name = "topic" + to_string(1);
            auto pub_node1 = std::make_shared<Publisher>(topic_name, real_period1);
            executor.add_node(pub_node1);

            int real_period2 = perd(gen);
            topic_name = "topic" + to_string(2);
            auto pub_node2 = std::make_shared<Publisher>(topic_name, real_period2);
            executor.add_node(pub_node2);

            int real_period3 = perd(gen);
            topic_name = "topic" + to_string(3);
            auto pub_node3 = std::make_shared<Publisher>(topic_name, real_period3);
            executor.add_node(pub_node3);

            int real_period4 = perd(gen);
            topic_name = "topic" + to_string(4);
            auto pub_node4 = std::make_shared<Publisher>(topic_name, real_period4);
            executor.add_node(pub_node4);

            auto sub_node = std::make_shared<SubscriberTopic4>(real_period1, real_period2, real_period3, real_period4, BOUND_ms, length_ms, BOUND_Between_ms, outfile_apr_, outfile_seam_);
            executor.add_node(sub_node);

            executor.spin();
            executor.remove_node(pub_node1);
            executor.remove_node(pub_node2);
            executor.remove_node(pub_node3);
            executor.remove_node(pub_node4);
            executor.remove_node(sub_node);

            cout << " We have finished test " << count + 1 << endl;
            count++;
        }
        break;
    }
    case 5:
    {
        int count = 0;
        while(count < test_num)
        {
            rclcpp::init(argc, argv);
            // rclcpp::executors::SingleThreadedExecutor executor;
            rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(),50);

            int real_period1 = perd(gen);
            topic_name = "topic" + to_string(1);
            auto pub_node1 = std::make_shared<Publisher>(topic_name, real_period1);
            executor.add_node(pub_node1);

            int real_period2 = perd(gen);
            topic_name = "topic" + to_string(2);
            auto pub_node2 = std::make_shared<Publisher>(topic_name, real_period2);
            executor.add_node(pub_node2);

            int real_period3 = perd(gen);
            topic_name = "topic" + to_string(3);
            auto pub_node3 = std::make_shared<Publisher>(topic_name, real_period3);
            executor.add_node(pub_node3);

            int real_period4 = perd(gen);
            topic_name = "topic" + to_string(4);
            auto pub_node4 = std::make_shared<Publisher>(topic_name, real_period4);
            executor.add_node(pub_node4);

            int real_period5 = perd(gen);
            topic_name = "topic" + to_string(5);
            auto pub_node5 = std::make_shared<Publisher>(topic_name, real_period5);
            executor.add_node(pub_node5);

            auto sub_node = std::make_shared<SubscriberTopic5>(real_period1, real_period2, real_period3, real_period4, real_period5, BOUND_ms, length_ms, BOUND_Between_ms, outfile_apr_, outfile_seam_);
            executor.add_node(sub_node);

            executor.spin();
            executor.remove_node(pub_node1);
            executor.remove_node(pub_node2);
            executor.remove_node(pub_node3);
            executor.remove_node(pub_node4);
            executor.remove_node(pub_node5);
            executor.remove_node(sub_node);

            cout << " We have finished test " << count + 1 << endl;
            count++;
        }
        break;
    }
    case 6:
    {
        int count = 0;
        while(count < test_num)
        {
            rclcpp::init(argc, argv);
            // rclcpp::executors::SingleThreadedExecutor executor;
            rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(),50);

            int real_period1 = perd(gen);
            topic_name = "topic" + to_string(1);
            auto pub_node1 = std::make_shared<Publisher>(topic_name, real_period1);
            executor.add_node(pub_node1);

            int real_period2 = perd(gen);
            topic_name = "topic" + to_string(2);
            auto pub_node2 = std::make_shared<Publisher>(topic_name, real_period2);
            executor.add_node(pub_node2);

            int real_period3 = perd(gen);
            topic_name = "topic" + to_string(3);
            auto pub_node3 = std::make_shared<Publisher>(topic_name, real_period3);
            executor.add_node(pub_node3);

            int real_period4 = perd(gen);
            topic_name = "topic" + to_string(4);
            auto pub_node4 = std::make_shared<Publisher>(topic_name, real_period4);
            executor.add_node(pub_node4);

            int real_period5 = perd(gen);
            topic_name = "topic" + to_string(5);
            auto pub_node5 = std::make_shared<Publisher>(topic_name, real_period5);
            executor.add_node(pub_node5);

            int real_period6 = perd(gen);
            topic_name = "topic" + to_string(6);
            auto pub_node6 = std::make_shared<Publisher>(topic_name, real_period6);
            executor.add_node(pub_node6);

            auto sub_node = std::make_shared<SubscriberTopic6>(real_period1, real_period2, real_period3, real_period4, real_period5, real_period6, BOUND_ms, length_ms, BOUND_Between_ms, outfile_apr_, outfile_seam_);
            executor.add_node(sub_node);

            executor.spin();

            executor.remove_node(pub_node1);
            executor.remove_node(pub_node2);
            executor.remove_node(pub_node3);
            executor.remove_node(pub_node4);
            executor.remove_node(pub_node5);
            executor.remove_node(pub_node6);
            executor.remove_node(sub_node);

            cout << " We have finished test " << count + 1 << endl;
            count++;
        }
        break;
    }
    case 7:
    {
        int count = 0;
        while(count < test_num)
        {
            rclcpp::init(argc, argv);
            // rclcpp::executors::SingleThreadedExecutor executor;
            rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(),50);

            int real_period1 = perd(gen);
            topic_name = "topic" + to_string(1);
            auto pub_node1 = std::make_shared<Publisher>(topic_name, real_period1);
            executor.add_node(pub_node1);

            int real_period2 = perd(gen);
            topic_name = "topic" + to_string(2);
            auto pub_node2 = std::make_shared<Publisher>(topic_name, real_period2);
            executor.add_node(pub_node2);

            int real_period3 = perd(gen);
            topic_name = "topic" + to_string(3);
            auto pub_node3 = std::make_shared<Publisher>(topic_name, real_period3);
            executor.add_node(pub_node3);

            int real_period4 = perd(gen);
            topic_name = "topic" + to_string(4);
            auto pub_node4 = std::make_shared<Publisher>(topic_name, real_period4);
            executor.add_node(pub_node4);

            int real_period5 = perd(gen);
            topic_name = "topic" + to_string(5);
            auto pub_node5 = std::make_shared<Publisher>(topic_name, real_period5);
            executor.add_node(pub_node5);

            int real_period6 = perd(gen);
            topic_name = "topic" + to_string(6);
            auto pub_node6 = std::make_shared<Publisher>(topic_name, real_period6);
            executor.add_node(pub_node6);

            int real_period7 = perd(gen);
            topic_name = "topic" + to_string(7);
            auto pub_node7 = std::make_shared<Publisher>(topic_name, real_period7);
            executor.add_node(pub_node7);

            auto sub_node = std::make_shared<SubscriberTopic7>(real_period1, real_period2, real_period3, real_period4, real_period5, real_period6, real_period7, BOUND_ms, length_ms, BOUND_Between_ms, outfile_apr_, outfile_seam_);
            executor.add_node(sub_node);

            executor.spin();
            executor.remove_node(pub_node1);
            executor.remove_node(pub_node2);
            executor.remove_node(pub_node3);
            executor.remove_node(pub_node4);
            executor.remove_node(pub_node5);
            executor.remove_node(pub_node6);
            executor.remove_node(pub_node7);
            executor.remove_node(sub_node);

            cout << " We have finished test " << count + 1 << endl;
            count++;
        }
        break;
    }
    case 8:
    {
        int count = 0;
        while(count < test_num)
        {
            rclcpp::init(argc, argv);
            // rclcpp::executors::SingleThreadedExecutor executor;
            rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(),50);

            int real_period1 = perd(gen);
            topic_name = "topic" + to_string(1);
            auto pub_node1 = std::make_shared<Publisher>(topic_name, real_period1);
            executor.add_node(pub_node1);

            int real_period2 = perd(gen);
            topic_name = "topic" + to_string(2);
            auto pub_node2 = std::make_shared<Publisher>(topic_name, real_period2);
            executor.add_node(pub_node2);

            int real_period3 = perd(gen);
            topic_name = "topic" + to_string(3);
            auto pub_node3 = std::make_shared<Publisher>(topic_name, real_period3);
            executor.add_node(pub_node3);

            int real_period4 = perd(gen);
            topic_name = "topic" + to_string(4);
            auto pub_node4 = std::make_shared<Publisher>(topic_name, real_period4);
            executor.add_node(pub_node4);

            int real_period5 = perd(gen);
            topic_name = "topic" + to_string(5);
            auto pub_node5 = std::make_shared<Publisher>(topic_name, real_period5);
            executor.add_node(pub_node5);

            int real_period6 = perd(gen);
            topic_name = "topic" + to_string(6);
            auto pub_node6 = std::make_shared<Publisher>(topic_name, real_period6);
            executor.add_node(pub_node6);

            int real_period7 = perd(gen);
            topic_name = "topic" + to_string(7);
            auto pub_node7 = std::make_shared<Publisher>(topic_name, real_period7);
            executor.add_node(pub_node7);

            int real_period8 = perd(gen);
            topic_name = "topic" + to_string(8);
            auto pub_node8 = std::make_shared<Publisher>(topic_name, real_period8);
            executor.add_node(pub_node8);

            auto sub_node = std::make_shared<SubscriberTopic8>(real_period1, real_period2, real_period3, real_period4, real_period5, real_period6, real_period7, real_period8, BOUND_ms, length_ms, BOUND_Between_ms, outfile_apr_, outfile_seam_);
            executor.add_node(sub_node);

            executor.spin();
            executor.remove_node(pub_node1);
            executor.remove_node(pub_node2);
            executor.remove_node(pub_node3);
            executor.remove_node(pub_node4);
            executor.remove_node(pub_node5);
            executor.remove_node(pub_node6);
            executor.remove_node(pub_node7);
            executor.remove_node(pub_node8);
            executor.remove_node(sub_node);

            cout << " We have finished test " << count + 1 << endl;
            count++;
        }            
        break;
    }
    case 9:
    {
        int count = 0;
        while(count < test_num)
        {        
            rclcpp::init(argc, argv);
            // rclcpp::executors::SingleThreadedExecutor executor;
            rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(),50);

            int real_period1 = perd(gen);
            topic_name = "topic" + to_string(1);
            auto pub_node1 = std::make_shared<Publisher>(topic_name, real_period1);
            executor.add_node(pub_node1);

            int real_period2 = perd(gen);
            topic_name = "topic" + to_string(2);
            auto pub_node2 = std::make_shared<Publisher>(topic_name, real_period2);
            executor.add_node(pub_node2);

            int real_period3 = perd(gen);
            topic_name = "topic" + to_string(3);
            auto pub_node3 = std::make_shared<Publisher>(topic_name, real_period3);
            executor.add_node(pub_node3);

            int real_period4 = perd(gen);
            topic_name = "topic" + to_string(4);
            auto pub_node4 = std::make_shared<Publisher>(topic_name, real_period4);
            executor.add_node(pub_node4);

            int real_period5 = perd(gen);
            topic_name = "topic" + to_string(5);
            auto pub_node5 = std::make_shared<Publisher>(topic_name, real_period5);
            executor.add_node(pub_node5);

            int real_period6 = perd(gen);
            topic_name = "topic" + to_string(6);
            auto pub_node6 = std::make_shared<Publisher>(topic_name, real_period6);
            executor.add_node(pub_node6);

            int real_period7 = perd(gen);
            topic_name = "topic" + to_string(7);
            auto pub_node7 = std::make_shared<Publisher>(topic_name, real_period7);
            executor.add_node(pub_node7);

            int real_period8 = perd(gen);
            topic_name = "topic" + to_string(8);
            auto pub_node8 = std::make_shared<Publisher>(topic_name, real_period8);
            executor.add_node(pub_node8);

            int real_period9 = perd(gen);
            topic_name = "topic" + to_string(9);
            auto pub_node9 = std::make_shared<Publisher>(topic_name, real_period9);
            executor.add_node(pub_node9);

            auto sub_node = std::make_shared<SubscriberTopic9>(real_period1, real_period2, real_period3, real_period4, real_period5, real_period6, real_period7, real_period8, real_period9, BOUND_ms, length_ms, BOUND_Between_ms, outfile_apr_, outfile_seam_);
            executor.add_node(sub_node);

            executor.spin();
            executor.remove_node(pub_node1);
            executor.remove_node(pub_node2);
            executor.remove_node(pub_node3);
            executor.remove_node(pub_node4);
            executor.remove_node(pub_node5);
            executor.remove_node(pub_node6);
            executor.remove_node(pub_node7);
            executor.remove_node(pub_node8);
            executor.remove_node(pub_node9);
            executor.remove_node(sub_node);

            cout << " We have finished test " << count + 1 << endl;
            count++;
        }            
        break;
    }
    default:
        break;        
    }

    success_rate_apr = (double)suc_apr / (double)count_apr;
    success_rate_seam = (double)suc_seam / (double)count_seam;
    cout << success_rate_apr << endl;
    cout << success_rate_seam << endl;
    outfile_sr_ << success_rate_apr << " " << success_rate_seam << endl;
    rclcpp::shutdown();
    return 0;
}
