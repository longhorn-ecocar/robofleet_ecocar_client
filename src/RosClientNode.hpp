#pragma once

#include <flatbuffers/flatbuffers.h>
#include <ros/ros.h>
#include <schema_generated.h>
#include <sensor_msgs/NavSatFix.h>

#include <QObject>
#include <chrono>
#include <thread>
#include <unordered_map>

#include "decode_ros.hpp"
#include "encode_ros.hpp"
#include "topic_config.hpp"

class RosClientNode : public QObject {
  Q_OBJECT

  struct TopicParams {
    double priority;
    double rate_limit;
    bool no_drop;
  };

  ros::AsyncSpinner spinner = ros::AsyncSpinner(1);
  ros::NodeHandle n;

  using TopicString = std::string;
  using MsgTypeString = std::string;
  std::vector<TopicString> pub_remote_topics;
  std::unordered_map<TopicString, ros::Subscriber> subs;
  std::unordered_map<TopicString, ros::Publisher> pubs;
  std::unordered_map<TopicString, TopicParams> topic_params;
  std::unordered_map<
      MsgTypeString, std::function<void(const QByteArray&, const TopicString&)>>
      pub_fns;

  const int verbosity_;

  /**
   * @brief Emit a ros_message_encoded() signal given a message and metadata.
   *
   * @tparam T type of msg
   * @param msg message to encode
   * @param msg_type type of the message; can be obtained using
   * ros::message_traits::DataType
   * @param from_topic topic on which message was received
   * @param to_topic remote topic to send to
   */
  template <typename T>
  void encode_ros_msg(
      const T& msg, const std::string& msg_type, const std::string& from_topic,
      const std::string& to_topic, const TopicParams& params) {
    // encode message
    flatbuffers::FlatBufferBuilder fbb;
    auto metadata = encode_metadata(fbb, msg_type, to_topic);
    auto root_offset = encode<T>(fbb, msg, metadata);
    fbb.Finish(flatbuffers::Offset<void>(root_offset));
    const QByteArray data{
        reinterpret_cast<const char*>(fbb.GetBufferPointer()),
        static_cast<int>(fbb.GetSize())};
    Q_EMIT ros_message_encoded(
        QString::fromStdString(to_topic),
        data,
        params.priority,
        params.rate_limit,
        params.no_drop);
  }

  template <typename T>
  void publish_ros_msg(
      const T& msg, const std::string& msg_type, const std::string& to_topic) {
    pubs[msg_type].publish(msg);
  }
  /**
   * @brief Set up pub/sub for a particular message type and topic.
   *
   * Creates a subscriber to the given topic, and emits ros_message_encoded()
   * signals for each message received.
   * @tparam T the ROS message type
   * @param from_topic the topic to subscribe to
   * @param to_topic the topic to publish to the server
   * @param max_publish_rate_hz the maximum number of messages per second to
   * publish on this topic
   */
  template <typename T>
  void register_local_msg_type(
      const std::string& from_topic, const std::string& to_topic) {
    // apply remapping to encode full topic name
    const std::string full_from_topic = ros::names::resolve(from_topic);
    const std::string full_to_topic = ros::names::resolve(to_topic);
    const std::string& msg_type = ros::message_traits::DataType<T>().value();

    if (subs.count(full_from_topic) > 0) {
      throw std::runtime_error(
          "Trying to register topic that is already registered. Topics must be "
          "unique.");
    }

    if (verbosity_ > 0) {
      printf(
          "Publishing Local Messages: %s [%s]->%s\n",
          full_from_topic.c_str(),
          msg_type.c_str(),
          full_to_topic.c_str());
    }

    // create subscription
    // have to use boost function because of how roscpp is implemented
    boost::function<void(T)> subscriber_handler =
        [this, msg_type, full_from_topic, full_to_topic](T msg) {
          encode_ros_msg<T>(
              msg,
              msg_type,
              full_from_topic,
              full_to_topic,
              topic_params[full_from_topic]);
        };
    subs[full_from_topic] =
        n.subscribe<T>(full_from_topic, 1, subscriber_handler);
  }

  /**
   * @brief Set up remote publishing for a particular message type and topic
   * sent from the server.
   *
   * Sets up the client to publish to `to_topic` whenever it
   * recieves a message of type `from_topic` from the remote server.
   * @tparam T the ROS message type
   * @param from_topic the remote topic to expect
   * @param to_topic the local topic to publish to
   */
  template <typename T>
  void register_remote_msg_type(
      const std::string& from_topic, const std::string& to_topic) {
    // apply remapping to encode full topic name
    const std::string full_from_topic = ros::names::resolve(from_topic);
    const std::string full_to_topic = ros::names::resolve(to_topic);
    const std::string& msg_type = ros::message_traits::DataType<T>().value();

    if (subs.count(full_to_topic) > 0) {
      std::cerr << "Publishing to a topic " << full_to_topic
                << " that is registered as a subscription. "
                << "Ensure this is intentional, as it can cause infinite "
                   "feedback loops."
                << std::endl;
    }

    if (verbosity_ > 0) {
      printf(
          "Receiving Remote Messages: %s [%s]->%s\n",
          full_from_topic.c_str(),
          msg_type.c_str(),
          full_to_topic.c_str());
    }

    // create function that will decode and publish a T message to any topic
    if (pub_fns.count(msg_type) == 0) {
      pub_fns[msg_type] = [this, full_to_topic](
                              const QByteArray& data,
                              const std::string& msg_type) {
        const auto* root =
            flatbuffers::GetRoot<typename flatbuffers_type_for<T>::type>(
                data.data());
        const T msg = decode<T>(root);
        publish_ros_msg<T>(msg, msg_type, full_to_topic);
      };
    }

    // Set up publishers for these remote messages
    if (pubs.count(full_from_topic) == 0) {
      pubs[full_from_topic] = n.advertise<T>(full_to_topic, 1);
    }

    pub_remote_topics.push_back(full_from_topic);
  }

 Q_SIGNALS:
  void ros_message_encoded(
      const QString& topic, const QByteArray& data, double priority,
      double rate_limit, bool no_drop);

 public Q_SLOTS:
  /**
   * @brief Attempt to decode an publish message data.
   *
   * Must call register_msg_type<T> before a message of type T can be decoded.
   * @param data the Flatbuffer-encoded message data
   */
  void decode_net_message(const QByteArray& data) {
    // extract metadata
    const fb::MsgWithMetadata* msg =
        flatbuffers::GetRoot<fb::MsgWithMetadata>(data.data());
    const std::string& msg_type = msg->__metadata()->type()->str();
    const std::string& topic = msg->__metadata()->topic()->str();

    // try to publish
    if (pub_fns.count(msg_type) == 0) {
      // if you get this unexpectedly, ensure that:
      // 1) you have registered the message type you intend to receive in your
      // configuration, and 2) you are sending a valid, fully-qualified message
      // type name (if you manually construct the flatbuffer)
      if (verbosity_ > 0) {
        std::cerr << "ignoring message of unregistered type " << msg_type
                  << std::endl;
      }
      return;
    }

    if (verbosity_ > 1) {
      std::cout << "Received message of type " << msg_type << " on topic "
                << topic << std::endl;
    }

    pub_fns[msg_type](data, topic);
  }

 public:
  /**
   * @brief subscribe to remote messages that were specified in the config by
   * calling register_remote_msg_type. This function should run once a websocket
   * connection has been established
   */
  void subscribe_remote_msgs() {
    for (auto topic : pub_remote_topics) {
      // Now, subscribe to the appropriate remote message
      amrl_msgs::RobofleetSubscription sub_msg;
      sub_msg.action = amrl_msgs::RobofleetSubscription::ACTION_SUBSCRIBE;
      sub_msg.topic_regex = topic;
      TopicParams params;
      params.no_drop = true;
      params.priority = 1.0;
      params.rate_limit = 1.0;
      encode_ros_msg<amrl_msgs::RobofleetSubscription>(
          sub_msg,
          ros::message_traits::DataType<amrl_msgs::RobofleetSubscription>()
              .value(),
          "/subscriptions",
          "/subscriptions",
          params);
    }
  }

  // Add a new template method to register a remote command type.
  template <typename T>
  void register_remote_command(
      const std::string& from_topic, const std::string& to_topic) {
    // Resolve full topic names.
    const std::string full_from_topic = ros::names::resolve(from_topic);
    const std::string full_to_topic = ros::names::resolve(to_topic);
    const std::string& msg_type = ros::message_traits::DataType<T>().value();

    // Advertise a local publisher for commands.
    if (pubs.count(full_to_topic) == 0) {
      pubs[full_to_topic] = n.advertise<T>(full_to_topic, 1);
    }

    // Create a function that decodes the remote command and publishes it
    // locally. (This is similar to what register_remote_msg_type does.)
    if (pub_fns.count(msg_type) == 0) {
      pub_fns[msg_type] = [this, full_to_topic](
                              const QByteArray& data,
                              const std::string& msg_type) {
        const auto* root =
            flatbuffers::GetRoot<typename flatbuffers_type_for<T>::type>(
                data.data());
        T msg = decode<T>(root);
        // Publish locally on the "to" topic.
        pubs[full_to_topic].publish(msg);
      };
    }
  }

  /**
   * @brief Register new listeners based on the given topic configuration.
   */
  template <typename Config>
  void configure(const Config& config);

  template <typename T>
  void configure(const topic_config::SendLocalTopic<T>& config) {
    config.assert_valid();
    register_local_msg_type<T>(config.from, config.to);
    const std::string full_from_topic = ros::names::resolve(config.from);
    topic_params[full_from_topic] = TopicParams{
        config.priority,
        config.rate_limit_hz.is_set() ? config.rate_limit_hz : 0.0,
        config.no_drop};
  }

  template <typename T>
  void configure(const topic_config::ReceiveRemoteTopic<T>& config) {
    config.assert_valid();
    register_remote_msg_type<T>(config.from, config.to);
  }

  RosClientNode(int verbosity) : verbosity_(verbosity) {
    // run forever
    spinner.start();
    if (verbosity_ > 0) {
      std::cout << "Started ROS Node" << std::endl;
    }
  }
};
