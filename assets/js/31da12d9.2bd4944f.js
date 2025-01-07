"use strict";(self.webpackChunkROS2_Beginners=self.webpackChunkROS2_Beginners||[]).push([[4934],{3164:(e,n,r)=>{r.r(n),r.d(n,{assets:()=>d,contentTitle:()=>o,default:()=>m,frontMatter:()=>s,metadata:()=>a,toc:()=>l});const a=JSON.parse('{"id":"node-settings/parameter","title":"ROS 2 Parameters","description":"Introduction","source":"@site/docs/06_node-settings/02_parameter.md","sourceDirName":"06_node-settings","slug":"/node-settings/parameter","permalink":"/ROS2-Beginners/docs/node-settings/parameter","draft":false,"unlisted":false,"editUrl":"https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/docs/06_node-settings/02_parameter.md","tags":[],"version":"current","sidebarPosition":2,"frontMatter":{},"sidebar":"tutorialSidebar","previous":{"title":"Introduction","permalink":"/ROS2-Beginners/docs/node-settings/intro"},"next":{"title":"Declaring Parameters in ROS 2","permalink":"/ROS2-Beginners/docs/node-settings/declare-parameter"}}');var i=r(4848),t=r(8453);const s={},o="ROS 2 Parameters",d={},l=[{value:"Introduction",id:"introduction",level:2},{value:"Understanding the Problem",id:"understanding-the-problem",level:2},{value:"Introduction to ROS 2 Parameters",id:"introduction-to-ros-2-parameters",level:2},{value:"Declaring Parameters in ROS 2",id:"declaring-parameters-in-ros-2",level:2},{value:"Example: Camera Driver Node",id:"example-camera-driver-node",level:3},{value:"Setting Parameter Values at Runtime",id:"setting-parameter-values-at-runtime",level:2},{value:"Using a YAML File",id:"using-a-yaml-file",level:3},{value:"Using Command-Line Arguments",id:"using-command-line-arguments",level:3},{value:"Recap",id:"recap",level:2},{value:"Summary",id:"summary",level:2}];function c(e){const n={code:"code",h1:"h1",h2:"h2",h3:"h3",header:"header",li:"li",p:"p",pre:"pre",ul:"ul",...(0,t.R)(),...e.components};return(0,i.jsxs)(i.Fragment,{children:[(0,i.jsx)(n.header,{children:(0,i.jsx)(n.h1,{id:"ros-2-parameters",children:"ROS 2 Parameters"})}),"\n",(0,i.jsx)(n.h2,{id:"introduction",children:"Introduction"}),"\n",(0,i.jsx)(n.p,{children:"In ROS 2 (Robot Operating System 2), parameters play a crucial role in configuring nodes without modifying the source code. This guide will provide an in-depth understanding of ROS 2 parameters, their purpose, and how to utilize them effectively in your projects."}),"\n",(0,i.jsx)(n.h2,{id:"understanding-the-problem",children:"Understanding the Problem"}),"\n",(0,i.jsx)(n.p,{children:"Consider a camera driver node within a ROS 2 package. This node is responsible for interfacing with a camera, capturing images, and possibly processing them. In a typical scenario, the node might need various configuration settings, such as:"}),"\n",(0,i.jsxs)(n.ul,{children:["\n",(0,i.jsx)(n.li,{children:"The name of the USB device to which the camera is connected."}),"\n",(0,i.jsx)(n.li,{children:"Frame rate settings (e.g., 30 fps or 60 fps)."}),"\n",(0,i.jsx)(n.li,{children:"Operating mode (e.g., simulation or real mode)."}),"\n"]}),"\n",(0,i.jsx)(n.p,{children:"Hardcoding these settings into your code is not ideal. Each time you need to change a setting, you would have to modify the code and recompile it. This approach is inflexible and inefficient, especially when you want to run multiple instances of the node with different configurations."}),"\n",(0,i.jsx)(n.h2,{id:"introduction-to-ros-2-parameters",children:"Introduction to ROS 2 Parameters"}),"\n",(0,i.jsx)(n.p,{children:"ROS 2 parameters provide a solution to this problem. They allow you to configure node settings dynamically at runtime without changing the source code. This flexibility makes it easier to manage and deploy nodes with different configurations."}),"\n",(0,i.jsx)(n.h2,{id:"declaring-parameters-in-ros-2",children:"Declaring Parameters in ROS 2"}),"\n",(0,i.jsx)(n.p,{children:"Before using parameters, you need to declare them in your node. Let's explore this with an example of a camera driver node."}),"\n",(0,i.jsx)(n.h3,{id:"example-camera-driver-node",children:"Example: Camera Driver Node"}),"\n",(0,i.jsx)(n.pre,{children:(0,i.jsx)(n.code,{className:"language-python",children:"import rclpy\nfrom rclpy.node import Node\nfrom rclpy.parameter import Parameter\n\nclass CameraDriverNode(Node):\n    def __init__(self):\n        super().__init__('camera_driver')\n\n        # Declare parameters\n        self.declare_parameter('usb_device', '/dev/video0')\n        self.declare_parameter('frame_rate', 30)\n        self.declare_parameter('simulation_mode', False)\n\n        # Retrieve parameter values\n        self.usb_device = self.get_parameter('usb_device').get_parameter_value().string_value\n        self.frame_rate = self.get_parameter('frame_rate').get_parameter_value().integer_value\n        self.simulation_mode = self.get_parameter('simulation_mode').get_parameter_value().bool_value\n\n        self.get_logger().info(f'USB Device: {self.usb_device}')\n        self.get_logger().info(f'Frame Rate: {self.frame_rate}')\n        self.get_logger().info(f'Simulation Mode: {self.simulation_mode}')\n\ndef main(args=None):\n    rclpy.init(args=args)\n    node = CameraDriverNode()\n    rclpy.spin(node)\n    node.destroy_node()\n    rclpy.shutdown()\n\nif __name__ == '__main__':\n    main()\n"})}),"\n",(0,i.jsxs)(n.p,{children:["In this example, the ",(0,i.jsx)(n.code,{children:"CameraDriverNode"})," class declares three parameters: ",(0,i.jsx)(n.code,{children:"usb_device"}),", ",(0,i.jsx)(n.code,{children:"frame_rate"}),", and ",(0,i.jsx)(n.code,{children:"simulation_mode"}),". These parameters have default values which can be overridden at runtime."]}),"\n",(0,i.jsx)(n.h2,{id:"setting-parameter-values-at-runtime",children:"Setting Parameter Values at Runtime"}),"\n",(0,i.jsx)(n.p,{children:"To run the node with specific parameter values, you can use a YAML file or command-line arguments."}),"\n",(0,i.jsx)(n.h3,{id:"using-a-yaml-file",children:"Using a YAML File"}),"\n",(0,i.jsxs)(n.p,{children:["Create a YAML file (e.g., ",(0,i.jsx)(n.code,{children:"camera_params.yaml"}),") with the following content:"]}),"\n",(0,i.jsx)(n.pre,{children:(0,i.jsx)(n.code,{className:"language-yaml",children:"camera_driver:\n  ros__parameters:\n    usb_device: '/dev/video1'\n    frame_rate: 60\n    simulation_mode: true\n"})}),"\n",(0,i.jsx)(n.p,{children:"Launch the node with the YAML file:"}),"\n",(0,i.jsx)(n.pre,{children:(0,i.jsx)(n.code,{className:"language-bash",children:"ros2 run your_package camera_driver_node --ros-args --params-file camera_params.yaml\n"})}),"\n",(0,i.jsx)(n.h3,{id:"using-command-line-arguments",children:"Using Command-Line Arguments"}),"\n",(0,i.jsx)(n.p,{children:"You can also set parameter values directly from the command line:"}),"\n",(0,i.jsx)(n.pre,{children:(0,i.jsx)(n.code,{className:"language-bash",children:"ros2 run your_package camera_driver_node --ros-args -p usb_device:=/dev/video1 -p frame_rate:=60 -p simulation_mode:=true\n"})}),"\n",(0,i.jsx)(n.h2,{id:"recap",children:"Recap"}),"\n",(0,i.jsx)(n.p,{children:"ROS 2 parameters provide a flexible and efficient way to configure nodes. They allow you to set configuration values at runtime, eliminating the need to modify and recompile code for different settings. Each parameter has a name and a data type, such as boolean, integer, double, string, or lists of these types."}),"\n",(0,i.jsx)(n.h2,{id:"summary",children:"Summary"}),"\n",(0,i.jsx)(n.p,{children:"In summary, ROS 2 parameters are essential for dynamically configuring nodes. They enhance the flexibility and manageability of your ROS 2 applications by allowing you to set and modify node settings at runtime. This guide has covered the basics of declaring and using parameters in ROS 2, providing you with the tools to implement this powerful feature in your projects."})]})}function m(e={}){const{wrapper:n}={...(0,t.R)(),...e.components};return n?(0,i.jsx)(n,{...e,children:(0,i.jsx)(c,{...e})}):c(e)}},8453:(e,n,r)=>{r.d(n,{R:()=>s,x:()=>o});var a=r(6540);const i={},t=a.createContext(i);function s(e){const n=a.useContext(t);return a.useMemo((function(){return"function"==typeof e?e(n):{...n,...e}}),[n,e])}function o(e){let n;return n=e.disableParentContext?"function"==typeof e.components?e.components(i):e.components||i:s(e.components),a.createElement(t.Provider,{value:n},e.children)}}}]);