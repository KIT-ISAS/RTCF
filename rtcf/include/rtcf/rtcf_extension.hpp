#ifndef RTCF_EXTENSION_H
#define RTCF_EXTENSION_H

#include <rtt/os/tlsf/tlsf.h>

// other
#include <ros/ros.h>
#include <rtt_ros/rtt_ros.h>

// this is a workaround for old log4cpp version that do not compile in C++17 due to usage of deprecated features
#define throw(...)
// these includes must be included before ocl/Category.hpp
//#include <log4cpp/Category.hh>
//#include <log4cpp/Priority.hh>
#include <log4cpp/HierarchyMaintainer.hh>
#include <ocl/Category.hpp>

#undef throw
#include <ocl/LoggingService.hpp>

#ifndef OS_RT_MALLOC
#warning "Logging needs rtalloc!"
#endif

#include <rtt/deployment/ComponentLoader.hpp>

struct memorySize {
  public:
    memorySize() : size(0) {}
    memorySize(size_t s) : size(s) {}
    size_t size;
};

class TLSFMemoryPool {
  public:
    /// Create default object (no allocation to pool)
    TLSFMemoryPool() : rtMem(0) {}
    /// Shutdown and deallocate memory pool (if necessary)
    ~TLSFMemoryPool() { shutdown(); }

    /** Initialize the default TLSF memory pool
     **
     ** @param memSize Size of memory pool
     ** @return true if successful
     ** @post if succesful then rtMem!=0 otherwise and rtMem=0
     */
    bool initialize(const size_t memSize) {
        if (0 != rtMem) return false;    // avoid double init
        if (0 >= memSize) return false;  // invalid size

        // don't calloc() as is first thing TLSF does.
        rtMem = malloc(memSize);
        assert(0 != rtMem);
        const size_t freeMem = init_memory_pool(memSize, rtMem);
        if ((size_t)-1 == freeMem) {
            std::cerr << std::dec << "Invalid memory pool size of " << memSize
                      << " bytes (TLSF has a several kilobyte overhead)." << std::endl;
            free(rtMem);
            rtMem = 0;
            return false;
        }
        std::cout << std::dec << "Real-time memory: " << freeMem << " bytes free of " << memSize << " allocated."
                  << std::endl;
        return true;
    }

    /** Shutdown the default TLSF memory pool
     ** @post rtMem=0 and the default TLSF memory pool is no longer available
     */
    void shutdown() {
        if (0 != rtMem) {
            const size_t overhead = get_overhead_size(rtMem);
            std::cout << std::dec << "TLSF bytes allocated=" << get_pool_size(rtMem) << " overhead=" << overhead
                      << " max-used=" << (get_max_size(rtMem) - overhead)
                      << " still-allocated=" << (get_used_size(rtMem) - overhead) << std::endl;

            destroy_memory_pool(rtMem);
            free(rtMem);
            rtMem = 0;
        }
    }

  protected:
    /// Memory allocated for the pool
    void* rtMem;
};

class RtcfExtension {
    // allow some classes access to private parameters
    // reason: public setter methods are tempting for the component-developers
    friend class ComponentContainer;
    friend class MainContext;
    friend class RTRunner;

  private:
    // by using pointers the user will not accidentally use the node handle in the component constructor
    ros::NodeHandlePtr nh_;
    ros::NodeHandlePtr nh_private_;

    inline static double frequency_;
    inline static double period_;
    inline static ros::Time last_timestamp_;

    OCL::logging::Category* logger;
    RTT::TaskContext* logger_out;
    RTT::TaskContext* log_service;

  public:
    TLSFMemoryPool memoryPool;
    RtcfExtension() {
        memorySize rtallocMemorySize = 524288;
        memoryPool.initialize(rtallocMemorySize.size);
        last_timestamp_ = ros::Time(0);

        // rtt_ros::import("orocos-ocl-logging");
        RTT::ComponentLoader::Instance()->import("ocl", "");

        log4cpp::HierarchyMaintainer::set_category_factory(OCL::logging::Category::createOCLCategory);

        logger_out = RTT::ComponentLoader::Instance()->loadComponent("rt_logger", "OCL::logging::OstreamAppender");
        RTT::Activity* logger_out_activity = new RTT::Activity();
        logger_out_activity->setPeriod(0.1);
        logger_out->setActivity(logger_out_activity);

        log_service =
            RTT::ComponentLoader::Instance()->loadComponent("rt_logger_service", "OCL::logging::LoggingService");
        RTT::Activity* loggin_service_activity = new RTT::Activity();
        loggin_service_activity->setPeriod(0.1);
        log_service->setActivity(loggin_service_activity);

        RTT::PropertyBag* bag = log_service->properties();
        RTT::Property<RTT::PropertyBag>* bag_appenders =
            (RTT::Property<RTT::PropertyBag>*)bag->getProperty("Appenders");

        RTT::Property<std::string> prop("", "", "rt_logger");
        bag_appenders->value().addProperty(prop);

        log_service->addPeer(logger_out);
        log_service->configure();
        logger_out->configure();
        log_service->start();
        logger_out->start();

        // auto* pointer = &log4cpp::Category::getInstance("org.orocos.ocl.logging.tests.TestComponent");
        // logger =
        //     dynamic_cast<OCL::logging::Category*>(pointer);  // questionable cast, but dynamic cast seems to be
        //     wrong
        // std::cout << typeid(logger).name() << std::endl;
        // ROS_WARN("testo on log4cpp base object");
        // pointer->warn("test on log4cpp base object");
        // logger->error(RTT::rt_string("Had an error here"));
        // logger->debug(RTT::rt_string("Some debug data ..."));
        // double i = 0;
        // std::string test;
        // ROS_ERROR("Log output should appear here:");
        // ROS_ERROR_STREAM("is priotiry enabled: " << logger->isPriorityEnabled(log4cpp::Priority::WARN));
        // logger->getRTStream(log4cpp::Priority::WARN) << "Some debug data and a double value " << i << test;

        OCL::logging::Category* logger =
            dynamic_cast<OCL::logging::Category*>(&log4cpp::Category::getInstance("name.test"));
        logger->error(RTT::rt_string("Hello world"));
        logger->debug(RTT::rt_string(
            "This is an extremely long string. Lorem ipsum dolor sit amet, consetetur sadipscing elitr, sed diam "
            "nonumy eirmod tempor invidunt ut labore et dolore magna aliquyam erat, sed diam voluptua. At vero eos et "
            "accusam et justo duo dolores et ea rebum. Stet clita kasd gubergren, no sea takimata sanctus est Lorem "
            "ipsum dolor sit amet. Lorem ipsum dolor sit amet, consetetur sadipscing elitr, sed diam nonumy eirmod "
            "tempor invidunt ut labore et dolore magna aliquyam erat, sed diam voluptua. At vero eos et accusam et "
            "justo duo dolores et ea rebum. Stet clita kasd gubergren, no sea takimata sanctus est Lorem ipsum dolor "
            "sit amet."));

        logger->getRTStream(log4cpp::Priority::DEBUG) << "debug message";
        logger->getRTStream(log4cpp::Priority::WARN) << "debug message";
        logger->getRTStream(log4cpp::Priority::INFO) << "debug message" << 1;
        logger->getRTStream(log4cpp::Priority::ERROR) << "debug message" << 1;
    };
    virtual ~RtcfExtension(){};

    const ros::NodeHandle& getNodeHandle() const { return *nh_; }
    const ros::NodeHandle& getPrivateNodeHandle() const { return *nh_private_; }

    const ros::Time& getTime() const { return last_timestamp_; }
    const double& getFrequency() const { return frequency_; }
    const double& getPeriod() const { return period_; }
};

#endif /* RTCF_EXTENSION_H */
