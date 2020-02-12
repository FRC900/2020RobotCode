class AlignActionAxisConfig
{
        public:
                AlignActionAxisConfig(const std::string &name,
                                                          const std::string &enable_pub_topic,
                                                          const std::string &command_pub_topic,
                                                          const std::string &error_sub_topic,
                                                          const std::string &timeout_param,
                                                          const std::string &error_threshold_param)
                        : name_(name)
                        , enable_pub_topic_(enable_pub_topic)
                        , command_pub_topic_(command_pub_topic)
                        , error_sub_topic_(error_sub_topic)
                        , timeout_param_(timeout_param)
                        , error_threshold_param_(error_threshold_param)
                {
                }
                std::string name_;
                std::string enable_pub_topic_;
                std::string command_pub_topic_;
                std::string error_sub_topic_;
                std::string timeout_param_;
                std::string error_threshold_param_;
};

class AlignActionAxisState
{
        public:
                AlignActionAxisState(const std::string &name,
                                          ros::NodeHandle &nh,
                                const std::string &enable_pub_topic,
                                const std::string &command_pub_topic,
                                const std::string &error_sub_topic,
                                const boost::function<void(const std_msgs::Float64MultiArrayConstPtr &msg)> &error_sub_cb,
                                const double timeout,
                                const double error_threshold)
                        : enable_pub_(nh.advertise<std_msgs::Bool>(enable_pub_topic, 1, true))
                        , command_pub_(nh.advertise<std_msgs::Bool>(command_pub_topic, 1, true))
                        , error_sub_(nh.subscribe(error_sub_topic, 1, error_sub_cb))
                        , aligned_(false)
                        , error_(0.0)
                        , timeout_(timeout)
                        , error_threshold_(error_threshold)
                        , timed_out_(false)
                {
                }
                ros::Publisher enable_pub_;
                ros::Publisher command_pub_;
                ros::Subscriber error_sub_;
                bool aligned_;
                double error_;
                double timeout_;
                double error_threshold_;
                bool timed_out_;
};

