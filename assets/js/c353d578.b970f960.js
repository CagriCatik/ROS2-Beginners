"use strict";(self.webpackChunkROS2_Beginners=self.webpackChunkROS2_Beginners||[]).push([[7813],{119:(e,n,a)=>{a.r(n),a.d(n,{assets:()=>o,contentTitle:()=>l,default:()=>h,frontMatter:()=>t,metadata:()=>i,toc:()=>c});const i=JSON.parse('{"id":"launch-files/launch-files","title":"ROS2 Launch Files","description":"Robot Operating System 2 (ROS2) is an open-source framework designed for building robot applications. It provides the necessary tools and libraries to develop, simulate, and deploy software for robotic systems. ROS2 is a reimagined version of ROS, with improvements in performance, security, and support for multiple platforms and programming languages.","source":"@site/docs/07_launch-files/02_launch-files.md","sourceDirName":"07_launch-files","slug":"/launch-files/launch-files","permalink":"/ROS2-Beginners/docs/launch-files/launch-files","draft":false,"unlisted":false,"editUrl":"https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/docs/07_launch-files/02_launch-files.md","tags":[],"version":"current","sidebarPosition":2,"frontMatter":{},"sidebar":"tutorialSidebar","previous":{"title":"Introduction","permalink":"/ROS2-Beginners/docs/launch-files/intro"},"next":{"title":"Creating and Using a Launch File","permalink":"/ROS2-Beginners/docs/launch-files/create-install-launch-file"}}');var r=a(4848),s=a(8453);const t={},l="ROS2 Launch Files",o={},c=[{value:"Understanding Launch Files",id:"understanding-launch-files",level:2},{value:"Problem Statement",id:"problem-statement",level:2},{value:"Example: Camera Driver Node",id:"example-camera-driver-node",level:2},{value:"Node Parameters",id:"node-parameters",level:3},{value:"Manual Node Launching",id:"manual-node-launching",level:2},{value:"Automating with Launch Files",id:"automating-with-launch-files",level:2},{value:"Creating a Launch File",id:"creating-a-launch-file",level:3},{value:"Launching the Application",id:"launching-the-application",level:2},{value:"Conclusion",id:"conclusion",level:2}];function d(e){const n={code:"code",h1:"h1",h2:"h2",h3:"h3",header:"header",li:"li",ol:"ol",p:"p",pre:"pre",strong:"strong",...(0,s.R)(),...e.components};return(0,r.jsxs)(r.Fragment,{children:[(0,r.jsx)(n.header,{children:(0,r.jsx)(n.h1,{id:"ros2-launch-files",children:"ROS2 Launch Files"})}),"\n",(0,r.jsx)(n.p,{children:"Robot Operating System 2 (ROS2) is an open-source framework designed for building robot applications. It provides the necessary tools and libraries to develop, simulate, and deploy software for robotic systems. ROS2 is a reimagined version of ROS, with improvements in performance, security, and support for multiple platforms and programming languages."}),"\n",(0,r.jsx)(n.h2,{id:"understanding-launch-files",children:"Understanding Launch Files"}),"\n",(0,r.jsx)(n.p,{children:"A launch file in ROS2 is an XML or Python script used to automate the launching of multiple nodes and the setting of their parameters and remappings. Launch files are essential for managing complex robotic systems where numerous nodes need to be started with specific configurations. By using launch files, developers can streamline the initialization process, avoid manual errors, and easily scale their applications."}),"\n",(0,r.jsx)(n.h2,{id:"problem-statement",children:"Problem Statement"}),"\n",(0,r.jsx)(n.p,{children:"Consider a scenario where you need to run multiple camera driver nodes on a robot, each with specific parameters and remappings. Manually starting each node in separate terminals is inefficient and prone to errors. This tutorial demonstrates how to use ROS2 launch files to automate this process, ensuring that all nodes are correctly configured and started with minimal effort."}),"\n",(0,r.jsx)(n.h2,{id:"example-camera-driver-node",children:"Example: Camera Driver Node"}),"\n",(0,r.jsx)(n.p,{children:"Let's explore the problem of starting multiple camera driver nodes with specific parameters and remappings. We will use a launch file to manage these nodes efficiently."}),"\n",(0,r.jsx)(n.h3,{id:"node-parameters",children:"Node Parameters"}),"\n",(0,r.jsx)(n.p,{children:"Each camera driver node requires three parameters:"}),"\n",(0,r.jsxs)(n.ol,{children:["\n",(0,r.jsxs)(n.li,{children:[(0,r.jsx)(n.strong,{children:"USB Device Name"}),": The name of the USB device."]}),"\n",(0,r.jsxs)(n.li,{children:[(0,r.jsx)(n.strong,{children:"Speed"}),": The speed setting for the camera."]}),"\n",(0,r.jsxs)(n.li,{children:[(0,r.jsx)(n.strong,{children:"Simulation Mode Flag"}),": A boolean flag indicating whether the node is running in simulation mode."]}),"\n"]}),"\n",(0,r.jsx)(n.p,{children:"Additionally, we need to remap the name of the node."}),"\n",(0,r.jsx)(n.h2,{id:"manual-node-launching",children:"Manual Node Launching"}),"\n",(0,r.jsx)(n.p,{children:"Manually launching nodes involves opening multiple terminals and starting each node with its parameters and remappings. Here\u2019s an example of launching a single camera driver node manually:"}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-bash",children:'ros2 run camera_driver camera_node --ros-args \\\n    -p usb_device:="/dev/video0" \\\n    -p speed:=30 \\\n    -p simulation:=False \\\n    --remap __node:=camera1\n'})}),"\n",(0,r.jsx)(n.p,{children:"To launch additional camera nodes and other nodes, you would repeat this process, modifying the parameters and remappings accordingly. This quickly becomes unmanageable with more nodes."}),"\n",(0,r.jsx)(n.h2,{id:"automating-with-launch-files",children:"Automating with Launch Files"}),"\n",(0,r.jsx)(n.p,{children:"A launch file simplifies the process by specifying all nodes, parameters, and remappings in a single file. Here\u2019s how to create and use a ROS2 launch file."}),"\n",(0,r.jsx)(n.h3,{id:"creating-a-launch-file",children:"Creating a Launch File"}),"\n",(0,r.jsx)(n.p,{children:"First, create a directory for your launch files if it doesn't exist:"}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-bash",children:"mkdir -p ~/ros2_ws/src/my_robot/launch\n"})}),"\n",(0,r.jsxs)(n.p,{children:["Create a new Python launch file, ",(0,r.jsx)(n.code,{children:"robot_launch.py"}),", in the ",(0,r.jsx)(n.code,{children:"launch"})," directory:"]}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-python",children:"from launch import LaunchDescription\nfrom launch_ros.actions import Node\n\ndef generate_launch_description():\n    return LaunchDescription([\n        Node(\n            package='camera_driver',\n            executable='camera_node',\n            name='camera1',\n            parameters=[{'usb_device': '/dev/video0', 'speed': 30, 'simulation': False}]\n        ),\n        Node(\n            package='camera_driver',\n            executable='camera_node',\n            name='camera2',\n            parameters=[{'usb_device': '/dev/video1', 'speed': 30, 'simulation': False}]\n        ),\n        Node(\n            package='camera_driver',\n            executable='camera_node',\n            name='camera3',\n            parameters=[{'usb_device': '/dev/video2', 'speed': 30, 'simulation': False}]\n        ),\n        Node(\n            package='robot_station',\n            executable='station_node',\n            name='station',\n            parameters=[{'param1': 'value1'}]\n        ),\n        Node(\n            package='panel_driver',\n            executable='panel_node',\n            name='panel1',\n            parameters=[{'param1': 'value2'}]\n        ),\n        Node(\n            package='panel_driver',\n            executable='panel_node',\n            name='panel2',\n            parameters=[{'param1': 'value3'}]\n        ),\n    ])\n"})}),"\n",(0,r.jsx)(n.h2,{id:"launching-the-application",children:"Launching the Application"}),"\n",(0,r.jsx)(n.p,{children:"To launch the entire application using the launch file, execute the following command in your terminal:"}),"\n",(0,r.jsx)(n.pre,{children:(0,r.jsx)(n.code,{className:"language-bash",children:"ros2 launch my_robot robot_launch.py\n"})}),"\n",(0,r.jsx)(n.p,{children:"This command starts all the nodes specified in the launch file with their respective parameters and remappings."}),"\n",(0,r.jsx)(n.h2,{id:"conclusion",children:"Conclusion"}),"\n",(0,r.jsx)(n.p,{children:"ROS2 launch files are a powerful tool for managing complex robotic applications. By defining nodes, parameters, and remappings in a single file, developers can automate the startup process, reduce manual errors, and easily scale their systems. This tutorial demonstrated the creation and use of a launch file to efficiently manage multiple camera driver nodes and other components of a robotic system."})]})}function h(e={}){const{wrapper:n}={...(0,s.R)(),...e.components};return n?(0,r.jsx)(n,{...e,children:(0,r.jsx)(d,{...e})}):d(e)}},8453:(e,n,a)=>{a.d(n,{R:()=>t,x:()=>l});var i=a(6540);const r={},s=i.createContext(r);function t(e){const n=i.useContext(s);return i.useMemo((function(){return"function"==typeof e?e(n):{...n,...e}}),[n,e])}function l(e){let n;return n=e.disableParentContext?"function"==typeof e.components?e.components(r):e.components||r:t(e.components),i.createElement(s.Provider,{value:n},e.children)}}}]);