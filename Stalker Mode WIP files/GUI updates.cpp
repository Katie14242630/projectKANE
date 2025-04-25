#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"  // For publishing point coordinates
#include <string>
#include <map>
#include <iostream>
#include <chrono>
#include <thread>

//stalker mode additions
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <cmath>

struct Book {
    std::string title;
    bool available;
    float x;  // X-coordinate in the map
    float y;  // Y-coordinate in the map
    std::string description;  // stalker mode addition for description
};

class GalleryDatabaseNode : public rclcpp::Node {
public:
    GalleryDatabaseNode() : Node("gallery_database_node") {
        initialize_books();
        // print_available_books();  // <-- REMOVED so we don't show gallery immediately

        // Publisher for borrowing books
        borrow_publisher_ = this->create_publisher<std_msgs::msg::String>("borrow_book", 10);

        // Publisher for custom point topic in Rviz
        point_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("book_goal_point", 10);

        // Publisher for navigation points for the robot
        navigate_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("navigate_to_book", 10);

        // Initialize the last borrow time
        last_borrow_time_ = this->now();

        // Start the user input thread
        user_input_thread_ = std::thread(&GalleryDatabaseNode::handle_user_input, this);

        //stalker mode additions
        //subscriber for human goal pose from HUman_detection_node processing
        human_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/human_goal_pose", 10,
            std::bind(&GalleryDatabaseNode::human_pose_callback, this, std::placeholders::_1));

        //subscriber to KANEbot pose
        amcl_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                current_robot_pose_ = msg->pose.pose;
            });

        //publisher to info response for GUI selection
        info_request_publisher_ = this->create_publisher<std_msgs::msg::String>("info_request", 10);

        //subscriber to info response for GUI selection 
        info_response_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/user_info_response", 10,
            std::bind(&GalleryDatabaseNode::info_response_callback, this, std::placeholders::_1));
            
    }

    ~GalleryDatabaseNode() {
        if (user_input_thread_.joinable()) {
            user_input_thread_.join();  // Ensure the thread is joined before destruction
        }
    }

private:
    std::map<std::string, Book> books_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr borrow_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr navigate_publisher_;
    rclcpp::Time last_borrow_time_; // To track the last time a borrow attempt was processed
    std::thread user_input_thread_;

    //stalker mode additions
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr human_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr info_request_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr info_response_sub_;
    geometry_msgs::msg::Pose current_robot_pose_;
    std::map<std::string, rclcpp::Time> last_info_time_;

    void initialize_books() {
        books_["Mona Lisa"]       = Book{"Mona Lisa", true, 7.0, 2.21535, "The original Mona Lisa painting measures 77 cm in height and 53 cm in width."};
        books_["Starry Night"]    = Book{"Starry Night", true, 7.0, 1.244027, "It depicts a night sky over a village, with a prominent cypress tree and swirling clouds."};
        books_["The Scream"]      = Book{"The Scream", true, 7.0, 0.77553, "Key work of Expressionism and Symbolism, exploring themes of anxiety and the human condition."};
        books_["American Gothic"] = Book{"American Gothic", true, 7.0, -1.61863, "Midwestern farmer and his daughter standing in front of their Carpenter Gothic style home."};
        books_["Nighthawks"]      = Book{"Nighthawks", true, 2.925824, -3.9, "Kaw Kaw."};
        books_["Sumflowers"]      = Book{"Sumflowers", true, 3.69138, 0.2, "Addition and photosynthesis."};  
        books_["Olympia"]         = Book{"Olympia", true, 4.732185, -1.2, "Birthplace of the Olympic Games."};
        books_["The Bath"]        = Book{"The Bath", true, 2.69425, -1.69643, "Rubber Ducks."};
    }

    void print_available_books() {
        std::cout << "\nCurrent Gallery:\n";
        for (const auto& [title, book] : books_) {
            if (book.available) {
                std::cout << "- " << title << std::endl;
            }
        }
        std::cout << std::endl;
    }

    void borrow_book_callback(const std_msgs::msg::String::SharedPtr msg) {
        auto current_time = this->now();
        auto time_diff = current_time - last_borrow_time_;

        // Only proceed if the last message was processed more than 1 second ago
        if (time_diff.seconds() >= 1.0) {
            std::string book_title = msg->data;
            auto it = books_.find(book_title);

            // Check if the book is available and update availability
            if (it != books_.end() && it->second.available) {
                it->second.available = false;
                std::cout << "\nSuccessfully targeted artwork: " << book_title << std::endl;

                // Publish the book's location as a point
                publish_point(it->second.x, it->second.y, book_title);
            } else {
                std::cout << "\nArtwork unavailable or not found: " << book_title << std::endl;
            }

            // Print the updated list of available books after each borrow request
            print_available_books();

            // Update the last borrow time
            last_borrow_time_ = current_time;
        }
    }
    
    //stalker mode additions
    void human_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "New human pose received: (%.2f, %.2f)",
                    msg->pose.position.x, msg->pose.position.y);
    
        // Publish goal
        geometry_msgs::msg::PointStamped goal_msg;
        goal_msg.header.stamp = msg->header.stamp;
        goal_msg.header.frame_id = "map";
        goal_msg.point.x = msg->pose.position.x;
        goal_msg.point.y = msg->pose.position.y;
        goal_msg.point.z = 0.0;
    
        // Publish goal to navigation
        navigate_publisher_->publish(goal_msg);

        // To compute if the KANEbot is near an artwork 
        for (auto& [title, book] : books_) {
        float dx = msg->pose.position.x - book.x;
        float dy = msg->pose.position.y - book.y;
        float dist = std::sqrt(dx * dx + dy * dy);

        // Only offer info if within range
        if (dist < 0.5) {
            auto now = this->now();
            if (last_info_time_.find(title) == last_info_time_.end() ||
                (now - last_info_time_[title]).seconds() > 10.0) {
                
                std_msgs::msg::String info_msg;
                info_msg.data = "info_request:" + title;
                info_request_publisher_->publish(info_msg);

                last_info_time_[title] = now;  // Update timestamp
                break;  // Only offer one artwork at a time
            }
            }
        }
    }

    //info for response callback for output
    void info_response_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string data = msg->data;
        if (data.rfind("accept:", 0) == 0) {
            std::string title = data.substr(7);  // Extract title
            auto it = books_.find(title);
            if (it != books_.end()) {
                std::cout << "\n🎤 Info about " << title << ":\n" << it->second.description << std::endl;
            }
        } else if (data.rfind("decline:", 0) == 0) {
            std::string title = data.substr(8);
            std::cout << "User declined info about: " << title << std::endl;
        }
    }

    void publish_point(float x, float y, const std::string& book_title) {
        geometry_msgs::msg::PointStamped point_msg;
        point_msg.header.stamp = this->now();
        point_msg.header.frame_id = "map";  // Change to "odom" if necessary

        // Set point coordinates
        point_msg.point.x = x;
        point_msg.point.y = y;
        point_msg.point.z = 0.0;

        // Debug message to confirm point is being published
        std::cout << "Publishing point for: " << book_title
                  << " at coordinates (" << x << ", " << y << ")" << std::endl;

        // Publish the point for navigation to the robot
        navigate_publisher_->publish(point_msg);
        // Publish the point for visualization in Rviz
        point_publisher_->publish(point_msg);
    }

    void handle_user_input() {
        // -------------------------------------------------
        // 1) ASK FOR MODE (STALKER vs. GUIDE)
        // -------------------------------------------------
        std::string mode_choice;
        std::cout << "Choose your mode:\n"
                  << "1) STALKER MODE\n"
                  << "2) GUIDE MODE\n"
                  << "Enter your choice: ";
        std::getline(std::cin, mode_choice);

        if (mode_choice == "1") {
            std::cout << "\n Thank you for choosing STALKER MODE." << std::endl;
            std::cout << "The robot will now begin following the detected human." << std::endl;
            std::cout << "Press Ctrl+C to stop.\n" << std::endl;
    
            // Just keep the thread alive while the Nav2 goal updates via the human_pose_callback
            while (rclcpp::ok()) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
    
                // Assuming you have some method to check for nearby artworks in Stalker mode.
                // After detecting artwork in range, ask for info response.
                char response;
                std::cout << "Do you want to receive more information about the artwork? (Y/N): ";
                std::cin >> response;
                std::cin.ignore();  // Clear the buffer after the response
    
                if (response == 'Y' || response == 'y') {
                    // Send the accept response with the artwork title
                    std_msgs::msg::String info_msg;
                    info_msg.data = "accept:" + title; 
                    info_response_publisher_->publish(info_msg);
                    std::cout << "Information accepted!" << std::endl;
                } else if (response == 'N' || response == 'n') {
                    // Send the decline response
                    std_msgs::msg::String info_msg;
                    info_msg.data = "decline:" + title; 
                    info_response_publisher_->publish(info_msg);
                    std::cout << "Information declined!" << std::endl;
                } else {
                    std::cout << "Invalid response. Please enter 'Y' or 'N'." << std::endl;
                }
            }
            return;
        }
        else if (mode_choice == "2") {
            // ---------------------------------------------
            // GUIDE MODE: proceed with the original logic
            // ---------------------------------------------
            print_available_books();
            while (rclcpp::ok()) {
                std::cout << "Hello Art Enthist =)\n"
                          << "Enter title(s) (comma-separated): ";
                std::string line;
                std::getline(std::cin, line);
                if (line.empty()) {
                    std::cout << "No input. Exiting guide mode.\n";
                    break;
                }
 
                // split & trim
                std::vector<std::string> titles;
                std::stringstream ss(line);
                std::string item;
                while (std::getline(ss, item, ',')) {
                    auto start = item.find_first_not_of(" \t");
                    auto end   = item.find_last_not_of(" \t");
                    if (start != std::string::npos && end != std::string::npos) {
                        titles.push_back(item.substr(start, end - start + 1));
                    }
                }
 
                // process
                for (const auto &t : titles) {
                    auto msg = std_msgs::msg::String();
                    msg.data = t;
                    borrow_publisher_->publish(msg);
                    borrow_book_callback(std::make_shared<std_msgs::msg::String>(msg));
                }
 
                print_available_books();
                char again;
                std::cout << "See more? (Y/N): ";
                std::cin >> again;
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                if (again!='Y' && again!='y') {
                    std::cout << "Exiting guide mode.\n";
                    break;
                }
                std::cout << "\033[2J\033[1;1H";  // clear
                print_available_books();
            }
        }
        else {
            std::cout << "Invalid choice. Exiting.\n";
            return;
        }
    }
};  // ← end of class

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    // Create the node and spin in a separate thread
    auto gallery_database_node = std::make_shared<GalleryDatabaseNode>();
    std::thread spin_thread([&]() { rclcpp::spin(gallery_database_node); });

    // Wait for user input
    std::cout << "Press Ctrl+C to exit." << std::endl;

    // Join the spin thread when done
    if (spin_thread.joinable()) {
        spin_thread.join();
    }

    rclcpp::shutdown();
    return 0;
}
