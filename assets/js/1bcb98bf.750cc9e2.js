"use strict";(self.webpackChunkmy_website=self.webpackChunkmy_website||[]).push([[4526],{2670:(e,n,s)=>{s.r(n),s.d(n,{assets:()=>o,contentTitle:()=>c,default:()=>u,frontMatter:()=>a,metadata:()=>i,toc:()=>l});const i=JSON.parse('{"id":"services/intro","title":"Services","description":"Introduction","source":"@site/docs/04_services/01_intro.md","sourceDirName":"04_services","slug":"/services/intro","permalink":"/ROS2-Beginners/docs/services/intro","draft":false,"unlisted":false,"editUrl":"https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/docs/04_services/01_intro.md","tags":[],"version":"current","sidebarPosition":1,"frontMatter":{},"sidebar":"tutorialSidebar","previous":{"title":"Services","permalink":"/ROS2-Beginners/docs/category/services"},"next":{"title":"ROS 2 Services","permalink":"/ROS2-Beginners/docs/services/definition"}}');var r=s(4848),t=s(8453);const a={},c="Services",o={},l=[{value:"Introduction",id:"introduction",level:2},{value:"Understanding ROS2 Services",id:"understanding-ros2-services",level:2},{value:"What are ROS2 Services?",id:"what-are-ros2-services",level:3},{value:"When to Use ROS2 Services",id:"when-to-use-ros2-services",level:3},{value:"Real-Life Analogy",id:"real-life-analogy",level:3},{value:"Writing Your Own Service",id:"writing-your-own-service",level:2},{value:"Python Implementation",id:"python-implementation",level:3},{value:"Server Node",id:"server-node",level:4},{value:"Client Node",id:"client-node",level:4},{value:"C++ Implementation",id:"c-implementation",level:3},{value:"Server Node",id:"server-node-1",level:4},{value:"Client Node",id:"client-node-1",level:4},{value:"Debugging Services from the Terminal",id:"debugging-services-from-the-terminal",level:2},{value:"Checking Available Services",id:"checking-available-services",level:3},{value:"Calling a Service",id:"calling-a-service",level:3},{value:"Describing a Service",id:"describing-a-service",level:3},{value:"Practical Activity",id:"practical-activity",level:2},{value:"Conclusion",id:"conclusion",level:2}];function d(e){const n={code:"code",h1:"h1",h2:"h2",h3:"h3",h4:"h4",header:"header",li:"li",ol:"ol",p:"p",pre:"pre",strong:"strong",ul:"ul",...(0,t.R)(),...e.components};return(0,r.jsxs)(r.Fragment,{children:[(0,r.jsx)(n.header,{children:(0,r.jsx)(n.h1,{id:"services",children:"Services"})}),"\n",(0,r.jsx)(n.h2,{id:"introduction",children:"Introduction"}),"\n",(0,r.jsx)(n.p,{children:"ROS2 (Robot Operating System 2) provides essential communication mechanisms that facilitate interaction between various nodes in a robotic system. The two primary communication features in ROS2 are Topics and Services. Topics are utilized for continuous data streams, while Services enable client/server interactions, allowing nodes to request and provide specific services."}),"\n",(0,r.jsx)(n.p,{children:"This tutorial focuses on ROS2 Services. By the end, you will be proficient in implementing and utilizing ROS2 Services for node communication. This will be achieved through the following steps:"}),"\n",(0,r.jsxs)(n.ol,{children:["\n",(0,r.jsx)(n.li,{children:"Understanding ROS2 Services and their appropriate use cases."}),"\n",(0,r.jsx)(n.li,{children:"Writing and implementing a Service (both client and server) in Python and C++."}),"\n",(0,r.jsx)(n.li,{children:"Debugging Services using terminal commands."}),"\n",(0,r.jsx)(n.li,{children:"Completing a practical activity to reinforce the concepts learned."}),"\n"]}),"\n",(0,r.jsx)(n.h2,{id:"understanding-ros2-services",children:"Understanding ROS2 Services"}),"\n",(0,r.jsx)(n.h3,{id:"what-are-ros2-services",children:"What are ROS2 Services?"}),"\n",(0,r.jsx)(n.p,{children:"ROS2 Services provide a synchronous communication mechanism, enabling nodes to send a request and wait for a response. This is analogous to a function call in programming, where a client node requests a service, and a server node processes this request and returns a response."}),"\n",(0,r.jsx)(n.h3,{id:"when-to-use-ros2-services",children:"When to Use ROS2 Services"}),"\n",(0,r.jsx)(n.p,{children:"Services are ideal for operations that require a response before proceeding. Examples include requesting the status of a sensor, commanding an actuator to move to a specific position, or fetching specific data from a node. Unlike Topics, which are used for continuous data streaming, Services are suitable for discrete transactions where an immediate response is necessary."}),"\n",(0,r.jsx)(n.h3,{id:"real-life-analogy",children:"Real-Life Analogy"}),"\n",(0,r.jsx)(n.p,{children:"Consider a restaurant scenario:"}),"\n",(0,r.jsxs)(n.ul,{children:["\n",(0,r.jsxs)(n.li,{children:[(0,r.jsx)(n.strong,{children:"Topic:"})," A waiter continuously updates the chef with the current orders and specials in a stream of information."]}),"\n",(0,r.jsxs)(n.li,{children:[(0,r.jsx)(n.strong,{children:"Service:"})," A customer requests the bill from the waiter, who then provides the bill after processing the request."]}),"\n"]}),"\n",(0,r.jsx)(n.h2,{id:"writing-your-own-service",children:"Writing Your Own Service"}),"\n",(0,r.jsx)(n.h3,{id:"python-implementation",children:"Python Implementation"}),"\n",(0,r.jsx)(n.h4,{id:"server-node",children:"Server Node"}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-python",children:"import rclpy\nfrom rclpy.node import Node\nfrom example_interfaces.srv import AddTwoInts\n\nclass AddTwoIntsServer(Node):\n    def __init__(self):\n        super().__init__('add_two_ints_server')\n        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)\n\n    def add_two_ints_callback(self, request, response):\n        response.sum = request.a + request.b\n        self.get_logger().info('Incoming request\\na: %d b: %d' % (request.a, request.b))\n        self.get_logger().info('Sending back response: %d' % (response.sum))\n        return response\n\ndef main(args=None):\n    rclpy.init(args=args)\n    node = AddTwoIntsServer()\n    rclpy.spin(node)\n    rclpy.shutdown()\n\nif __name__ == '__main__':\n    main()\n"})}),"\n",(0,r.jsx)(n.h4,{id:"client-node",children:"Client Node"}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-python",children:"import sys\nimport rclpy\nfrom rclpy.node import Node\nfrom example_interfaces.srv import AddTwoInts\n\nclass AddTwoIntsClient(Node):\n    def __init__(self):\n        super().__init__('add_two_ints_client')\n        self.cli = self.create_client(AddTwoInts, 'add_two_ints')\n        while not self.cli.wait_for_service(timeout_sec=1.0):\n            self.get_logger().info('service not available, waiting again...')\n        self.req = AddTwoInts.Request()\n\n    def send_request(self):\n        self.req.a = int(sys.argv[1])\n        self.req.b = int(sys.argv[2])\n        self.future = self.cli.call_async(self.req)\n        rclpy.spin_until_future_complete(self, self.future)\n        return self.future.result()\n\ndef main(args=None):\n    rclpy.init(args=args)\n    client = AddTwoIntsClient()\n    response = client.send_request()\n    client.get_logger().info('Result of add_two_ints: %d' % response.sum)\n    rclpy.shutdown()\n\nif __name__ == '__main__':\n    main()\n"})}),"\n",(0,r.jsx)(n.h3,{id:"c-implementation",children:"C++ Implementation"}),"\n",(0,r.jsx)(n.h4,{id:"server-node-1",children:"Server Node"}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-cpp",children:'#include "rclcpp/rclcpp.hpp"\n#include "example_interfaces/srv/add_two_ints.hpp"\n\nusing AddTwoInts = example_interfaces::srv::AddTwoInts;\nusing namespace std::placeholders;\n\nclass AddTwoIntsServer : public rclcpp::Node\n{\npublic:\n    AddTwoIntsServer() : Node("add_two_ints_server")\n    {\n        service_ = create_service<AddTwoInts>("add_two_ints", std::bind(&AddTwoIntsServer::handle_service, this, _1, _2));\n    }\n\nprivate:\n    void handle_service(const std::shared_ptr<AddTwoInts::Request> request,\n                        std::shared_ptr<AddTwoInts::Response> response)\n    {\n        response->sum = request->a + request->b;\n        RCLCPP_INFO(this->get_logger(), "Incoming request\\na: %ld b: %ld", request->a, request->b);\n        RCLCPP_INFO(this->get_logger(), "Sending back response: %ld", response->sum);\n    }\n\n    rclcpp::Service<AddTwoInts>::SharedPtr service_;\n};\n\nint main(int argc, char **argv)\n{\n    rclcpp::init(argc, argv);\n    auto node = std::make_shared<AddTwoIntsServer>();\n    rclcpp::spin(node);\n    rclcpp::shutdown();\n    return 0;\n}\n'})}),"\n",(0,r.jsx)(n.h4,{id:"client-node-1",children:"Client Node"}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-cpp",children:'#include "rclcpp/rclcpp.hpp"\n#include "example_interfaces/srv/add_two_ints.hpp"\n\nusing AddTwoInts = example_interfaces::srv::AddTwoInts;\nusing namespace std::chrono_literals;\n\nclass AddTwoIntsClient : public rclcpp::Node\n{\npublic:\n    AddTwoIntsClient() : Node("add_two_ints_client")\n    {\n        client_ = create_client<AddTwoInts>("add_two_ints");\n        while (!client_->wait_for_service(1s))\n        {\n            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");\n        }\n        request_ = std::make_shared<AddTwoInts::Request>();\n    }\n\n    void send_request(int64_t a, int64_t b)\n    {\n        request_->a = a;\n        request_->b = b;\n        using ServiceResponseFuture = rclcpp::Client<AddTwoInts>::SharedFuture;\n        auto response_received_callback = [this](ServiceResponseFuture future)\n        {\n            auto response = future.get();\n            RCLCPP_INFO(this->get_logger(), "Result of add_two_ints: %ld", response->sum);\n        };\n        auto future_result = client_->async_send_request(request_, response_received_callback);\n    }\n\nprivate:\n    rclcpp::Client<AddTwoInts>::SharedPtr client_;\n    std::shared_ptr<AddTwoInts::Request> request_;\n};\n\nint main(int argc, char **argv)\n{\n    rclcpp::init(argc, argv);\n    auto client = std::make_shared<AddTwoIntsClient>();\n    client->send_request(std::stoi(argv[1]), std::stoi(argv[2]));\n    rclcpp::spin(client);\n    rclcpp::shutdown();\n    return 0;\n}\n'})}),"\n",(0,r.jsx)(n.h2,{id:"debugging-services-from-the-terminal",children:"Debugging Services from the Terminal"}),"\n",(0,r.jsx)(n.h3,{id:"checking-available-services",children:"Checking Available Services"}),"\n",(0,r.jsx)(n.p,{children:"To list all available services, use the following command:"}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-bash",children:"ros2 service list\n"})}),"\n",(0,r.jsx)(n.h3,{id:"calling-a-service",children:"Calling a Service"}),"\n",(0,r.jsx)(n.p,{children:"To call a service and send a request from the terminal, use:"}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-bash",children:'ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 2, b: 3}"\n'})}),"\n",(0,r.jsx)(n.h3,{id:"describing-a-service",children:"Describing a Service"}),"\n",(0,r.jsx)(n.p,{children:"To get detailed information about a service, use:"}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-bash",children:"ros2 service type /add_two_ints\nros2 interface show example_interfaces/srv/AddTwoInts\n"})}),"\n",(0,r.jsx)(n.h2,{id:"practical-activity",children:"Practical Activity"}),"\n",(0,r.jsx)(n.p,{children:"Implement the provided Python and C++ service and client nodes. Once implemented, test the communication between the nodes. Use the terminal commands to debug and verify the functionality of your service. Modify the service to handle different types of requests and responses, such as strings or custom data types, to reinforce your understanding of ROS2 Services."}),"\n",(0,r.jsx)(n.h2,{id:"conclusion",children:"Conclusion"}),"\n",(0,r.jsx)(n.p,{children:"This tutorial has provided a comprehensive guide to understanding, implementing, and debugging ROS2 Services. Mastery of these concepts is crucial for developing robust and responsive robotic applications. Continue experimenting with different service types and explore advanced features to further enhance your skills in ROS2."})]})}function u(e={}){const{wrapper:n}={...(0,t.R)(),...e.components};return n?(0,r.jsx)(n,{...e,children:(0,r.jsx)(d,{...e})}):d(e)}},8453:(e,n,s)=>{s.d(n,{R:()=>a,x:()=>c});var i=s(6540);const r={},t=i.createContext(r);function a(e){const n=i.useContext(t);return i.useMemo((function(){return"function"==typeof e?e(n):{...n,...e}}),[n,e])}function c(e){let n;return n=e.disableParentContext?"function"==typeof e.components?e.components(r):e.components||r:a(e.components),i.createElement(t.Provider,{value:n},e.children)}}}]);