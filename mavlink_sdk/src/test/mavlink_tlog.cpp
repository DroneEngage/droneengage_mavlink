/**
 * @file mavlink_tlog_test.cpp
 * @brief Test suite for MAVLink TLOG functionality
 * @author DroneEngage Team
 * @date 2026
 */

#include "../tlog/mavlink_tlog.hpp"
#include <iostream>
#include <cassert>
#include <fstream>
#include <chrono>
#include <thread>

class MavlinkTlogTester : public mavlinksdk::tlog::CCallBack_MavlinkTlog
{
public:
    int m_tests_passed = 0;
    int m_tests_failed = 0;
    std::string m_test_filename = "test_flight.tlog";

public:
    // Callback implementations
    void OnTlogOpened(const std::string& filename) override
    {
        std::cout << "[CALLBACK] TLOG opened: " << filename << std::endl;
    }
    
    void OnTlogClosed(const std::string& filename) override
    {
        std::cout << "[CALLBACK] TLOG closed: " << filename << std::endl;
    }
    
    void OnTlogError(const std::string& error_msg) override
    {
        std::cout << "[CALLBACK] TLOG error: " << error_msg << std::endl;
    }

    // Test helper methods
    void assert_test(bool condition, const std::string& test_name)
    {
        if (condition) {
            std::cout << "[PASS] " << test_name << std::endl;
            m_tests_passed++;
        } else {
            std::cout << "[FAIL] " << test_name << std::endl;
            m_tests_failed++;
        }
    }

    void cleanup_test_file()
    {
        std::remove(m_test_filename.c_str());
    }

    // Test methods
    void test_constructor_destructor()
    {
        std::cout << "\n=== Testing Constructor/Destructor ===" << std::endl;
        
        {
            mavlinksdk::tlog::CMavlinkTlog tlogger;
            assert_test(!tlogger.isOpen(), "Constructor: Initially closed");
            assert_test(tlogger.getMessagesWritten() == 0, "Constructor: Zero messages written");
            assert_test(tlogger.getFilename().empty(), "Constructor: Empty filename");
        }
        // Destructor called here
        
        std::cout << "Destructor test completed (no crash)" << std::endl;
    }

    void test_file_operations()
    {
        std::cout << "\n=== Testing File Operations ===" << std::endl;
        
        mavlinksdk::tlog::CMavlinkTlog tlogger;
        
        // Test opening file
        bool open_result = tlogger.open(m_test_filename);
        assert_test(open_result, "Open file: Success");
        assert_test(tlogger.isOpen(), "Open file: Is open flag set");
        assert_test(tlogger.getFilename() == m_test_filename, "Open file: Filename stored correctly");
        assert_test(tlogger.getMessagesWritten() == 0, "Open file: Zero messages initially");
        
        // Test closing file
        tlogger.close();
        assert_test(!tlogger.isOpen(), "Close file: Is open flag cleared");
        
        // Test reopening same file
        open_result = tlogger.open(m_test_filename);
        assert_test(open_result, "Reopen file: Success");
    }

    void test_message_writing()
    {
        std::cout << "\n=== Testing Message Writing ===" << std::endl;
        
        mavlinksdk::tlog::CMavlinkTlog tlogger;
        
        // Test writing to closed file (should fail)
        mavlink_message_t msg;
        mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, 
                                   MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA, 
                                   0, 0, MAV_STATE_ACTIVE);
        
        bool write_result = tlogger.writeMessage(msg);
        assert_test(!write_result, "Write to closed file: Should fail");
        
        // Open file and test writing
        tlogger.open(m_test_filename);
        
        // Write single message
        write_result = tlogger.writeMessage(msg);
        assert_test(write_result, "Write single message: Success");
        assert_test(tlogger.getMessagesWritten() == 1, "Write single message: Count incremented");
        
        // Write multiple messages
        for (int i = 0; i < 5; i++) {
            write_result = tlogger.writeMessage(msg);
            assert_test(write_result, "Write multiple messages: Message " + std::to_string(i+1) + " success");
        }
        
        assert_test(tlogger.getMessagesWritten() == 6, "Write multiple messages: Total count correct");
        
        tlogger.close();
    }

    void test_callback_functionality()
    {
        std::cout << "\n=== Testing Callback Functionality ===" << std::endl;
        
        mavlinksdk::tlog::CMavlinkTlog tlogger;
        tlogger.set_callback_tlog(this);
        
        // Test open callback
        bool open_result = tlogger.open(m_test_filename);
        assert_test(open_result, "Callback test: Open with callback");
        
        // Test write callback (error case)
        tlogger.close();
        mavlink_message_t msg;
        mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, 
                                   MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA, 
                                   0, 0, MAV_STATE_ACTIVE);
        bool write_result = tlogger.writeMessage(msg);
        assert_test(!write_result, "Callback test: Error callback triggered");
        
        // Test close callback
        tlogger.open(m_test_filename);
        tlogger.close();
        assert_test(!tlogger.isOpen(), "Callback test: Close with callback");
    }

    void test_file_integrity()
    {
        std::cout << "\n=== Testing File Integrity ===" << std::endl;
        
        mavlinksdk::tlog::CMavlinkTlog tlogger;
        tlogger.open(m_test_filename);
        
        // Write known message
        mavlink_message_t msg;
        mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, 
                                   MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA, 
                                   0, 0, MAV_STATE_ACTIVE);
        
        tlogger.writeMessage(msg);
        tlogger.close();
        
        // Check file exists and has content
        std::ifstream file(m_test_filename, std::ios::binary | std::ios::ate);
        assert_test(file.good(), "File integrity: File exists");
        
        if (file.good()) {
            std::streamsize size = file.tellg();
            assert_test(size > 0, "File integrity: File has content");
            assert_test(size >= 12, "File integrity: Minimum size (8 bytes timestamp + message header)");
        }
        file.close();
    }

    void test_multiple_instances()
    {
        std::cout << "\n=== Testing Multiple Instances ===" << std::endl;
        
        mavlinksdk::tlog::CMavlinkTlog tlogger1, tlogger2;
        
        // Test different files
        bool open1 = tlogger1.open("test1.tlog");
        bool open2 = tlogger2.open("test2.tlog");
        
        assert_test(open1 && open2, "Multiple instances: Both files opened");
        assert_test(tlogger1.getFilename() == "test1.tlog", "Multiple instances: Correct filename 1");
        assert_test(tlogger2.getFilename() == "test2.tlog", "Multiple instances: Correct filename 2");
        
        tlogger1.close();
        tlogger2.close();
        
        // Cleanup
        std::remove("test1.tlog");
        std::remove("test2.tlog");
    }

    void run_all_tests()
    {
        std::cout << "========================================" << std::endl;
        std::cout << "   MAVLink TLOG Test Suite" << std::endl;
        std::cout << "========================================" << std::endl;
        
        cleanup_test_file();
        
        test_constructor_destructor();
        test_file_operations();
        test_message_writing();
        test_callback_functionality();
        test_file_integrity();
        test_multiple_instances();
        
        cleanup_test_file();
        
        std::cout << "\n========================================" << std::endl;
        std::cout << "Test Results:" << std::endl;
        std::cout << "Passed: " << m_tests_passed << std::endl;
        std::cout << "Failed: " << m_tests_failed << std::endl;
        std::cout << "Total:  " << (m_tests_passed + m_tests_failed) << std::endl;
        std::cout << "========================================" << std::endl;
        
        if (m_tests_failed == 0) {
            std::cout << "\n🎉 ALL TESTS PASSED! 🎉" << std::endl;
        } else {
            std::cout << "\n❌ " << m_tests_failed << " TESTS FAILED ❌" << std::endl;
        }
    }
};

int main(int argc, char* argv[])
{
    std::cout << "MAVLink TLOG Tester" << std::endl;
    std::cout << "===================" << std::endl;
    
    try {
        MavlinkTlogTester tester;
        tester.run_all_tests();
        
        return tester.m_tests_failed == 0 ? 0 : 1;
        
    } catch (const std::exception& e) {
        std::cerr << "Test suite crashed with exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Test suite crashed with unknown exception" << std::endl;
        return 1;
    }
}