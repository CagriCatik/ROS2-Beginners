"use strict";(self.webpackChunkmy_website=self.webpackChunkmy_website||[]).push([[2760],{6130:(e,n,i)=>{i.r(n),i.d(n,{assets:()=>a,contentTitle:()=>t,default:()=>h,frontMatter:()=>o,metadata:()=>s,toc:()=>c});const s=JSON.parse('{"id":"tools/intro","title":"ROS2 Tools","description":"Overview","source":"@site/docs/02_tools/01_intro.md","sourceDirName":"02_tools","slug":"/tools/intro","permalink":"/docs/tools/intro","draft":false,"unlisted":false,"editUrl":"https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/docs/02_tools/01_intro.md","tags":[],"version":"current","sidebarPosition":1,"frontMatter":{},"sidebar":"tutorialSidebar","previous":{"title":"Tools","permalink":"/docs/category/tools"},"next":{"title":"Creating and Managing ROS2 Nodes","permalink":"/docs/tools/debug-monitor-nodes"}}');var r=i(4848),l=i(8453);const o={},t="ROS2 Tools",a={},c=[{value:"Overview",id:"overview",level:2},{value:"Objectives",id:"objectives",level:2},{value:"Running Nodes with <code>ros2 run</code>",id:"running-nodes-with-ros2-run",level:2},{value:"Basic Node Execution",id:"basic-node-execution",level:3},{value:"Running with a Custom Node Name",id:"running-with-a-custom-node-name",level:3},{value:"Technical Notes",id:"technical-notes",level:3},{value:"Debugging Nodes with ROS2 CLI",id:"debugging-nodes-with-ros2-cli",level:2},{value:"Listing Active Nodes",id:"listing-active-nodes",level:3},{value:"Getting Node Information",id:"getting-node-information",level:3},{value:"Introspecting Topics",id:"introspecting-topics",level:3},{value:"Service Introspection",id:"service-introspection",level:3},{value:"Visualizing the ROS2 Graph with <code>rqt_graph</code>",id:"visualizing-the-ros2-graph-with-rqt_graph",level:2},{value:"Launching <code>rqt_graph</code>",id:"launching-rqt_graph",level:3},{value:"Interpreting the Graph",id:"interpreting-the-graph",level:3},{value:"Technical Notes",id:"technical-notes-1",level:3},{value:"Exploring Turtlesim",id:"exploring-turtlesim",level:2},{value:"Launching Turtlesim",id:"launching-turtlesim",level:3},{value:"Controlling Turtlesim",id:"controlling-turtlesim",level:3},{value:"Practical Activity",id:"practical-activity",level:2},{value:"Creating a New ROS2 Package",id:"creating-a-new-ros2-package",level:3},{value:"Step 1: Creating a Package",id:"step-1-creating-a-package",level:4},{value:"Step 2: Writing the Publisher Node (Python)",id:"step-2-writing-the-publisher-node-python",level:4},{value:"Step 3: Writing the Subscriber Node (Python)",id:"step-3-writing-the-subscriber-node-python",level:4},{value:"Step 4: Building and Running the Package",id:"step-4-building-and-running-the-package",level:4},{value:"Step 5: Using the Tools",id:"step-5-using-the-tools",level:4},{value:"Conclusion",id:"conclusion",level:2}];function d(e){const n={code:"code",h1:"h1",h2:"h2",h3:"h3",h4:"h4",header:"header",li:"li",ol:"ol",p:"p",pre:"pre",strong:"strong",ul:"ul",...(0,l.R)(),...e.components};return(0,r.jsxs)(r.Fragment,{children:[(0,r.jsx)(n.header,{children:(0,r.jsx)(n.h1,{id:"ros2-tools",children:"ROS2 Tools"})}),"\n",(0,r.jsx)(n.h2,{id:"overview",children:"Overview"}),"\n",(0,r.jsxs)(n.p,{children:["This tutorial covers some of the essential tools within the ROS2 (Robot Operating System 2) ecosystem, which are indispensable for developing, managing, and debugging ROS2 applications. By the end of this tutorial, you will have a comprehensive understanding of how to run ROS2 nodes, use the ROS2 Command Line Interface (CLI) for debugging, visualize the ROS2 computational graph using ",(0,r.jsx)(n.code,{children:"rqt_graph"}),", and explore basic simulation using Turtlesim."]}),"\n",(0,r.jsx)(n.h2,{id:"objectives",children:"Objectives"}),"\n",(0,r.jsx)(n.p,{children:"Upon completion of this tutorial, you will have the proficiency to:"}),"\n",(0,r.jsxs)(n.ol,{children:["\n",(0,r.jsx)(n.li,{children:"Execute ROS2 nodes with various options."}),"\n",(0,r.jsx)(n.li,{children:"Debug nodes using ROS2 CLI tools."}),"\n",(0,r.jsxs)(n.li,{children:["Visualize the computational graph of your ROS2 application using ",(0,r.jsx)(n.code,{children:"rqt_graph"}),"."]}),"\n",(0,r.jsx)(n.li,{children:"Utilize Turtlesim, a basic 2D simulation tool, to understand ROS2 concepts."}),"\n"]}),"\n",(0,r.jsxs)(n.h2,{id:"running-nodes-with-ros2-run",children:["Running Nodes with ",(0,r.jsx)(n.code,{children:"ros2 run"})]}),"\n",(0,r.jsxs)(n.p,{children:["The ",(0,r.jsx)(n.code,{children:"ros2 run"})," command is fundamental for launching nodes in ROS2. It is versatile, allowing you to execute nodes with different configurations, including changing the node's name dynamically. Below is a detailed guide on how to use ",(0,r.jsx)(n.code,{children:"ros2 run"})," effectively."]}),"\n",(0,r.jsx)(n.h3,{id:"basic-node-execution",children:"Basic Node Execution"}),"\n",(0,r.jsx)(n.p,{children:"To run a node in ROS2, use the following command:"}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-bash",children:"ros2 run <package_name> <node_executable>\n"})}),"\n",(0,r.jsx)(n.p,{children:(0,r.jsx)(n.strong,{children:"Example:"})}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-bash",children:"ros2 run demo_nodes_cpp talker\n"})}),"\n",(0,r.jsxs)(n.p,{children:["This command launches the ",(0,r.jsx)(n.code,{children:"talker"})," node from the ",(0,r.jsx)(n.code,{children:"demo_nodes_cpp"})," package. This node is a basic publisher that sends messages to a topic."]}),"\n",(0,r.jsx)(n.h3,{id:"running-with-a-custom-node-name",children:"Running with a Custom Node Name"}),"\n",(0,r.jsxs)(n.p,{children:["In ROS2, you can change the name of a node dynamically using the ",(0,r.jsx)(n.code,{children:"--ros-args"})," flag followed by the ",(0,r.jsx)(n.code,{children:"-r"})," option:"]}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-bash",children:"ros2 run <package_name> <node_executable> --ros-args -r __node:=custom_node_name\n"})}),"\n",(0,r.jsx)(n.p,{children:(0,r.jsx)(n.strong,{children:"Example:"})}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-bash",children:"ros2 run demo_nodes_cpp talker --ros-args -r __node:=my_custom_talker\n"})}),"\n",(0,r.jsxs)(n.p,{children:["This command runs the ",(0,r.jsx)(n.code,{children:"talker"})," node but renames it to ",(0,r.jsx)(n.code,{children:"my_custom_talker"}),"."]}),"\n",(0,r.jsx)(n.h3,{id:"technical-notes",children:"Technical Notes"}),"\n",(0,r.jsxs)(n.ul,{children:["\n",(0,r.jsxs)(n.li,{children:["The ",(0,r.jsx)(n.code,{children:"ros2 run"})," command locates the executable for the specified node within the specified package."]}),"\n",(0,r.jsxs)(n.li,{children:["The ",(0,r.jsx)(n.code,{children:"--ros-args"})," flag allows additional ROS-specific arguments, such as remapping names, setting parameters, or changing QoS settings, to be passed to the node at runtime."]}),"\n"]}),"\n",(0,r.jsx)(n.h2,{id:"debugging-nodes-with-ros2-cli",children:"Debugging Nodes with ROS2 CLI"}),"\n",(0,r.jsx)(n.p,{children:"The ROS2 CLI provides a suite of commands to interact with and debug nodes. These tools are essential for inspecting the state and behavior of your ROS2 nodes during development."}),"\n",(0,r.jsx)(n.h3,{id:"listing-active-nodes",children:"Listing Active Nodes"}),"\n",(0,r.jsx)(n.p,{children:"To list all currently running nodes, use:"}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-bash",children:"ros2 node list\n"})}),"\n",(0,r.jsx)(n.p,{children:"This command will display a list of node names currently active in the ROS2 system."}),"\n",(0,r.jsx)(n.h3,{id:"getting-node-information",children:"Getting Node Information"}),"\n",(0,r.jsx)(n.p,{children:"To get detailed information about a specific node, such as its publishers, subscribers, services, and parameters, use:"}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-bash",children:"ros2 node info <node_name>\n"})}),"\n",(0,r.jsx)(n.p,{children:(0,r.jsx)(n.strong,{children:"Example:"})}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-bash",children:"ros2 node info /my_custom_talker\n"})}),"\n",(0,r.jsxs)(n.p,{children:["This command will provide detailed information about the ",(0,r.jsx)(n.code,{children:"my_custom_talker"})," node, including its publishers, subscribers, services, and parameters."]}),"\n",(0,r.jsx)(n.h3,{id:"introspecting-topics",children:"Introspecting Topics"}),"\n",(0,r.jsx)(n.p,{children:"Topic introspection is critical for understanding the flow of messages between nodes. Here are some essential commands:"}),"\n",(0,r.jsxs)(n.ul,{children:["\n",(0,r.jsxs)(n.li,{children:["\n",(0,r.jsx)(n.p,{children:(0,r.jsx)(n.strong,{children:"Listing Topics:"})}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-bash",children:"ros2 topic list\n"})}),"\n",(0,r.jsx)(n.p,{children:"This command lists all active topics."}),"\n"]}),"\n",(0,r.jsxs)(n.li,{children:["\n",(0,r.jsx)(n.p,{children:(0,r.jsx)(n.strong,{children:"Displaying Topic Information:"})}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-bash",children:"ros2 topic info <topic_name>\n"})}),"\n",(0,r.jsx)(n.p,{children:(0,r.jsx)(n.strong,{children:"Example:"})}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-bash",children:"ros2 topic info /chatter\n"})}),"\n",(0,r.jsxs)(n.p,{children:["This command provides details about the ",(0,r.jsx)(n.code,{children:"/chatter"})," topic, including the type of messages being published and the number of publishers and subscribers."]}),"\n"]}),"\n",(0,r.jsxs)(n.li,{children:["\n",(0,r.jsx)(n.p,{children:(0,r.jsx)(n.strong,{children:"Echoing Topic Data:"})}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-bash",children:"ros2 topic echo <topic_name>\n"})}),"\n",(0,r.jsx)(n.p,{children:(0,r.jsx)(n.strong,{children:"Example:"})}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-bash",children:"ros2 topic echo /chatter\n"})}),"\n",(0,r.jsxs)(n.p,{children:["This command prints the messages being published on the ",(0,r.jsx)(n.code,{children:"/chatter"})," topic to the console, useful for monitoring live data."]}),"\n"]}),"\n"]}),"\n",(0,r.jsx)(n.h3,{id:"service-introspection",children:"Service Introspection"}),"\n",(0,r.jsx)(n.p,{children:"Services in ROS2 are used for synchronous communication between nodes. The following commands help in inspecting active services:"}),"\n",(0,r.jsxs)(n.ul,{children:["\n",(0,r.jsxs)(n.li,{children:["\n",(0,r.jsx)(n.p,{children:(0,r.jsx)(n.strong,{children:"Listing Services:"})}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-bash",children:"ros2 service list\n"})}),"\n",(0,r.jsx)(n.p,{children:"This command lists all active services in the ROS2 system."}),"\n"]}),"\n",(0,r.jsxs)(n.li,{children:["\n",(0,r.jsx)(n.p,{children:(0,r.jsx)(n.strong,{children:"Displaying Service Information:"})}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-bash",children:"ros2 service info <service_name>\n"})}),"\n",(0,r.jsx)(n.p,{children:(0,r.jsx)(n.strong,{children:"Example:"})}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-bash",children:"ros2 service info /spawn\n"})}),"\n",(0,r.jsxs)(n.p,{children:["This command displays information about the ",(0,r.jsx)(n.code,{children:"/spawn"})," service, including its request and response types."]}),"\n"]}),"\n"]}),"\n",(0,r.jsxs)(n.h2,{id:"visualizing-the-ros2-graph-with-rqt_graph",children:["Visualizing the ROS2 Graph with ",(0,r.jsx)(n.code,{children:"rqt_graph"})]}),"\n",(0,r.jsxs)(n.p,{children:["The ",(0,r.jsx)(n.code,{children:"rqt_graph"})," tool is invaluable for visualizing the computational graph of your ROS2 system. It provides a graphical representation of nodes, topics, and their interconnections, making it easier to understand and debug complex systems."]}),"\n",(0,r.jsxs)(n.h3,{id:"launching-rqt_graph",children:["Launching ",(0,r.jsx)(n.code,{children:"rqt_graph"})]}),"\n",(0,r.jsxs)(n.p,{children:["To open ",(0,r.jsx)(n.code,{children:"rqt_graph"}),", use the following command:"]}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-bash",children:"ros2 run rqt_graph rqt_graph\n"})}),"\n",(0,r.jsxs)(n.p,{children:["This will launch the ",(0,r.jsx)(n.code,{children:"rqt_graph"})," interface, displaying the nodes and their connections in the system."]}),"\n",(0,r.jsx)(n.h3,{id:"interpreting-the-graph",children:"Interpreting the Graph"}),"\n",(0,r.jsxs)(n.ul,{children:["\n",(0,r.jsxs)(n.li,{children:[(0,r.jsx)(n.strong,{children:"Nodes"})," are represented as rectangles in the graph."]}),"\n",(0,r.jsxs)(n.li,{children:[(0,r.jsx)(n.strong,{children:"Topics"})," are shown as ellipses."]}),"\n",(0,r.jsxs)(n.li,{children:[(0,r.jsx)(n.strong,{children:"Connections"})," between nodes and topics represent the flow of messages, with arrows indicating the direction of communication."]}),"\n"]}),"\n",(0,r.jsx)(n.h3,{id:"technical-notes-1",children:"Technical Notes"}),"\n",(0,r.jsxs)(n.ul,{children:["\n",(0,r.jsxs)(n.li,{children:[(0,r.jsx)(n.code,{children:"rqt_graph"})," relies on the underlying ROS2 graph information, which is dynamically updated as nodes are started and stopped."]}),"\n",(0,r.jsx)(n.li,{children:"This tool is especially useful in complex systems with numerous nodes and topics, allowing you to identify misconfigurations or unintended connections quickly."}),"\n"]}),"\n",(0,r.jsx)(n.h2,{id:"exploring-turtlesim",children:"Exploring Turtlesim"}),"\n",(0,r.jsx)(n.p,{children:"Turtlesim is a basic 2D simulation tool included with ROS2, often used for learning and demonstration purposes. It provides a simple environment where you can visualize and interact with basic ROS2 concepts."}),"\n",(0,r.jsx)(n.h3,{id:"launching-turtlesim",children:"Launching Turtlesim"}),"\n",(0,r.jsx)(n.p,{children:"To start the Turtlesim simulation, use:"}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-bash",children:"ros2 run turtlesim turtlesim_node\n"})}),"\n",(0,r.jsx)(n.p,{children:"This command launches the Turtlesim node, which opens a graphical window displaying the turtle."}),"\n",(0,r.jsx)(n.h3,{id:"controlling-turtlesim",children:"Controlling Turtlesim"}),"\n",(0,r.jsx)(n.p,{children:"To control the turtle using your keyboard, start the teleoperation node:"}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-bash",children:"ros2 run turtlesim turtle_teleop_key\n"})}),"\n",(0,r.jsx)(n.p,{children:"This command allows you to move the turtle around the screen using the arrow keys."}),"\n",(0,r.jsx)(n.h2,{id:"practical-activity",children:"Practical Activity"}),"\n",(0,r.jsx)(n.h3,{id:"creating-a-new-ros2-package",children:"Creating a New ROS2 Package"}),"\n",(0,r.jsx)(n.p,{children:"As a hands-on exercise, you will create a new ROS2 package, write simple publisher and subscriber nodes, and use the tools discussed above to debug and visualize your application."}),"\n",(0,r.jsx)(n.h4,{id:"step-1-creating-a-package",children:"Step 1: Creating a Package"}),"\n",(0,r.jsx)(n.p,{children:"Create a new ROS2 package using the following command:"}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-bash",children:"ros2 pkg create --build-type ament_cmake my_package\n"})}),"\n",(0,r.jsxs)(n.p,{children:["This command generates a new package named ",(0,r.jsx)(n.code,{children:"my_package"})," with the CMake build system."]}),"\n",(0,r.jsx)(n.h4,{id:"step-2-writing-the-publisher-node-python",children:"Step 2: Writing the Publisher Node (Python)"}),"\n",(0,r.jsxs)(n.p,{children:["Create a file named ",(0,r.jsx)(n.code,{children:"publisher_node.py"})," in the ",(0,r.jsx)(n.code,{children:"my_package/my_package"})," directory with the following content:"]}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-python",children:"import rclpy\nfrom rclpy.node import Node\nfrom std_msgs.msg import String\n\nclass MinimalPublisher(Node):\n    def __init__(self):\n        super().__init__('minimal_publisher')\n        self.publisher_ = self.create_publisher(String, 'topic', 10)\n        timer_period = 0.5  # seconds\n        self.timer = self.create_timer(timer_period, self.timer_callback)\n\n    def timer_callback(self):\n        msg = String()\n        msg.data = 'Hello, world!'\n        self.publisher_.publish(msg)\n\ndef main(args=None):\n    rclpy.init(args=args)\n    minimal_publisher = MinimalPublisher()\n    rclpy.spin(minimal_publisher)\n    minimal_publisher.destroy_node()\n    rclpy.shutdown()\n\nif __name__ == '__main__':\n    main()\n"})}),"\n",(0,r.jsx)(n.h4,{id:"step-3-writing-the-subscriber-node-python",children:"Step 3: Writing the Subscriber Node (Python)"}),"\n",(0,r.jsxs)(n.p,{children:["Create a file named ",(0,r.jsx)(n.code,{children:"subscriber_node.py"})," in the ",(0,r.jsx)(n.code,{children:"my_package/my_package"})," directory with the following content:"]}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-python",children:"import rclpy\nfrom rclpy.node import Node\nfrom std_msgs.msg import String\n\nclass MinimalSubscriber(Node):\n    def __init__(self):\n        super().__init__('minimal_subscriber')\n        self.subscription = self.create_subscription(\n            String,\n            'topic',\n            self.listener_callback,\n            10)\n        self.subscription  # prevent unused variable warning\n\n    def listener_callback(self, msg):\n        self.get_logger().info('I heard: \"%s\"' % msg.data)\n\ndef main(args=None):\n    rclpy.init(args=args)\n    minimal_subscriber = MinimalSubscriber()\n    rclpy.spin(minimal_subscriber)\n    minimal_subscriber.destroy_node()\n    rclpy.shutdown()\n\nif __name__ == '__main__':\n    main()\n"})}),"\n",(0,r.jsx)(n.h4,{id:"step-4-building-and-running-the-package",children:"Step 4: Building and Running the Package"}),"\n",(0,r.jsxs)(n.ul,{children:["\n",(0,r.jsxs)(n.li,{children:["\n",(0,r.jsx)(n.p,{children:(0,r.jsx)(n.strong,{children:"Build the package:"})}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-bash",children:"cd ~/ros2_ws\ncolcon build\n"})}),"\n"]}),"\n",(0,r.jsxs)(n.li,{children:["\n",(0,r.jsx)(n.p,{children:(0,r.jsx)(n.strong,{children:"Source the setup file:"})}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-bash",children:"source install/setup.bash\n"})}),"\n"]}),"\n",(0,r.jsxs)(n.li,{children:["\n",(0,r.jsx)(n.p,{children:(0,r.jsx)(n.strong,{children:"Run the publisher and subscriber:"})}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-bash",children:"ros2 run my_package publisher_node\nros2 run my_package subscriber_node\n"})}),"\n"]}),"\n"]}),"\n",(0,r.jsx)(n.h4,{id:"step-5-using-the-tools",children:"Step 5: Using the Tools"}),"\n",(0,r.jsxs)(n.ul,{children:["\n",(0,r.jsxs)(n.li,{children:["\n",(0,r.jsx)(n.p,{children:(0,r.jsx)(n.strong,{children:"List and get information about the nodes:"})}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-bash",children:"ros2 node list\nros2 node info /minimal_publisher\nros2 node info /minimal_subscriber\n"})}),"\n"]}),"\n",(0,r.jsxs)(n.li,{children:["\n",(0,r.jsx)(n.p,{children:(0,r.jsx)(n.strong,{children:"Visualize the communication graph:"})}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-bash",children:"ros2 run rqt_graph rqt_graph\n"})}),"\n"]}),"\n"]}),"\n",(0,r.jsx)(n.p,{children:"This will show how"}),"\n",(0,r.jsxs)(n.p,{children:["the ",(0,r.jsx)(n.code,{children:"minimal_publisher"})," and ",(0,r.jsx)(n.code,{children:"minimal_subscriber"})," nodes communicate via the ",(0,r.jsx)(n.code,{children:"topic"})," topic."]}),"\n",(0,r.jsx)(n.h2,{id:"conclusion",children:"Conclusion"}),"\n",(0,r.jsx)(n.p,{children:"By mastering these ROS2 tools, you are now equipped to develop, debug, and maintain robust ROS2 applications. Understanding how to run nodes with various options, use the ROS2 CLI for debugging, visualize node interactions, and experiment with Turtlesim will enable you to tackle more complex ROS2 projects with confidence."})]})}function h(e={}){const{wrapper:n}={...(0,l.R)(),...e.components};return n?(0,r.jsx)(n,{...e,children:(0,r.jsx)(d,{...e})}):d(e)}},8453:(e,n,i)=>{i.d(n,{R:()=>o,x:()=>t});var s=i(6540);const r={},l=s.createContext(r);function o(e){const n=s.useContext(l);return s.useMemo((function(){return"function"==typeof e?e(n):{...n,...e}}),[n,e])}function t(e){let n;return n=e.disableParentContext?"function"==typeof e.components?e.components(r):e.components||r:o(e.components),s.createElement(l.Provider,{value:n},e.children)}}}]);