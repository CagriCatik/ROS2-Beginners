"use strict";(self.webpackChunkmy_website=self.webpackChunkmy_website||[]).push([[619],{8769:(e,n,i)=>{i.r(n),i.d(n,{assets:()=>c,contentTitle:()=>r,default:()=>u,frontMatter:()=>a,metadata:()=>s,toc:()=>l});const s=JSON.parse('{"id":"tools/rename-node-runtime","title":"Launching Multiple Instances of a Node with Different Configurations","description":"In many robotic applications, there is often a need to launch multiple instances of the same node, each with different configurations. This tutorial will guide you through the process of launching the same node multiple times with unique configurations in ROS 2. We will address potential issues and explain best practices to ensure smooth operation of your nodes.","source":"@site/docs/02_tools/03_rename-node-runtime.md","sourceDirName":"02_tools","slug":"/tools/rename-node-runtime","permalink":"/docs/tools/rename-node-runtime","draft":false,"unlisted":false,"editUrl":"https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/docs/02_tools/03_rename-node-runtime.md","tags":[],"version":"current","sidebarPosition":3,"frontMatter":{},"sidebar":"tutorialSidebar","previous":{"title":"Creating and Managing ROS2 Nodes","permalink":"/docs/tools/debug-monitor-nodes"},"next":{"title":"Workspace Building and Efficient Development with Python Nodes","permalink":"/docs/tools/colcon"}}');var o=i(4848),t=i(8453);const a={},r="Launching Multiple Instances of a Node with Different Configurations",c={},l=[{value:"Background",id:"background",level:2},{value:"Practical Example: Multiple Temperature Sensors",id:"practical-example-multiple-temperature-sensors",level:2},{value:"Step-by-Step Tutorial",id:"step-by-step-tutorial",level:2},{value:"1. Launching Nodes with Identical Names",id:"1-launching-nodes-with-identical-names",level:3},{value:"2. Properly Launching Multiple Instances",id:"2-properly-launching-multiple-instances",level:3},{value:"3. Verifying Node Configurations",id:"3-verifying-node-configurations",level:3},{value:"4. Extending the Example: Remapping Topics and Services",id:"4-extending-the-example-remapping-topics-and-services",level:3},{value:"Best Practices",id:"best-practices",level:2},{value:"Conclusion",id:"conclusion",level:2}];function d(e){const n={code:"code",h1:"h1",h2:"h2",h3:"h3",header:"header",li:"li",ol:"ol",p:"p",pre:"pre",strong:"strong",...(0,t.R)(),...e.components};return(0,o.jsxs)(o.Fragment,{children:[(0,o.jsx)(n.header,{children:(0,o.jsx)(n.h1,{id:"launching-multiple-instances-of-a-node-with-different-configurations",children:"Launching Multiple Instances of a Node with Different Configurations"})}),"\n",(0,o.jsx)(n.p,{children:"In many robotic applications, there is often a need to launch multiple instances of the same node, each with different configurations. This tutorial will guide you through the process of launching the same node multiple times with unique configurations in ROS 2. We will address potential issues and explain best practices to ensure smooth operation of your nodes."}),"\n",(0,o.jsx)(n.h2,{id:"background",children:"Background"}),"\n",(0,o.jsx)(n.p,{children:"In ROS 1, it was not possible to launch multiple nodes with the same name. However, in ROS 2, it is technically possible, though not advisable due to the complications it introduces. Launching multiple nodes with the same name can lead to unintended side effects and communication problems within your ROS graph."}),"\n",(0,o.jsx)(n.h2,{id:"practical-example-multiple-temperature-sensors",children:"Practical Example: Multiple Temperature Sensors"}),"\n",(0,o.jsx)(n.p,{children:"Consider a scenario where you have a temperature sensor node and you need to launch this node five times, once for each of your five temperature sensors. Each instance must have a unique name to function correctly."}),"\n",(0,o.jsx)(n.h2,{id:"step-by-step-tutorial",children:"Step-by-Step Tutorial"}),"\n",(0,o.jsx)(n.h3,{id:"1-launching-nodes-with-identical-names",children:"1. Launching Nodes with Identical Names"}),"\n",(0,o.jsx)(n.p,{children:"First, let's explore what happens if you launch nodes with the same name. In a terminal, run:"}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{className:"language-sh",children:"ros2 run my_python_package my_node\n"})}),"\n",(0,o.jsx)(n.p,{children:"In a different terminal, run the same command again:"}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{className:"language-sh",children:"ros2 run my_python_package my_node\n"})}),"\n",(0,o.jsx)(n.p,{children:"Then list the active nodes:"}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{className:"language-sh",children:"ros2 node list\n"})}),"\n",(0,o.jsx)(n.p,{children:"You will see a warning:"}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{className:"language-sh",children:"Be aware that some nodes in the graph share an exact name and this can have unintended side effects.\n"})}),"\n",(0,o.jsx)(n.p,{children:"Both nodes will appear in the list, but this setup is problematic. Interactions with these nodes, such as retrieving node information, will yield warnings and ambiguous results."}),"\n",(0,o.jsx)(n.h3,{id:"2-properly-launching-multiple-instances",children:"2. Properly Launching Multiple Instances"}),"\n",(0,o.jsx)(n.p,{children:"To avoid issues, you must assign a unique name to each node instance. ROS 2 allows you to remap node names dynamically at launch. Here\u2019s how to do it."}),"\n",(0,o.jsx)(n.p,{children:"In one terminal, start the first node with a unique name:"}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{className:"language-sh",children:"ros2 run my_python_package my_node --ros-args -r __node:=sensor_node_1\n"})}),"\n",(0,o.jsx)(n.p,{children:"In another terminal, start the second node with a different unique name:"}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{className:"language-sh",children:"ros2 run my_python_package my_node --ros-args -r __node:=sensor_node_2\n"})}),"\n",(0,o.jsx)(n.p,{children:"Repeat this process for additional nodes, ensuring each has a unique name:"}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{className:"language-sh",children:"ros2 run my_python_package my_node --ros-args -r __node:=sensor_node_3\nros2 run my_python_package my_node --ros-args -r __node:=sensor_node_4\nros2 run my_python_package my_node --ros-args -r __node:=sensor_node_5\n"})}),"\n",(0,o.jsx)(n.p,{children:"Now, if you list the active nodes:"}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{className:"language-sh",children:"ros2 node list\n"})}),"\n",(0,o.jsx)(n.p,{children:"You will see all five nodes, each with a unique name, indicating that they are correctly set up."}),"\n",(0,o.jsx)(n.h3,{id:"3-verifying-node-configurations",children:"3. Verifying Node Configurations"}),"\n",(0,o.jsx)(n.p,{children:"To verify that each node is functioning as expected, you can query the node information. For instance:"}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{className:"language-sh",children:"ros2 node info sensor_node_1\nros2 node info sensor_node_2\n"})}),"\n",(0,o.jsx)(n.p,{children:"Ensure that each command returns information specific to the respective node instance."}),"\n",(0,o.jsx)(n.h3,{id:"4-extending-the-example-remapping-topics-and-services",children:"4. Extending the Example: Remapping Topics and Services"}),"\n",(0,o.jsx)(n.p,{children:"Beyond renaming nodes, ROS 2 allows you to remap topics, services, and parameters at runtime. This feature is useful for configuring nodes dynamically without changing the source code."}),"\n",(0,o.jsx)(n.p,{children:"For example, to remap a topic:"}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{className:"language-sh",children:"ros2 run my_python_package my_node --ros-args -r __node:=sensor_node_1 -r temperature:=sensor_1/temperature\n"})}),"\n",(0,o.jsxs)(n.p,{children:["This command remaps the ",(0,o.jsx)(n.code,{children:"temperature"})," topic to ",(0,o.jsx)(n.code,{children:"sensor_1/temperature"})," for the ",(0,o.jsx)(n.code,{children:"sensor_node_1"}),"."]}),"\n",(0,o.jsx)(n.h2,{id:"best-practices",children:"Best Practices"}),"\n",(0,o.jsxs)(n.ol,{children:["\n",(0,o.jsxs)(n.li,{children:[(0,o.jsx)(n.strong,{children:"Unique Naming"}),": Always assign unique names to node instances to avoid conflicts and unintended behaviors."]}),"\n",(0,o.jsxs)(n.li,{children:[(0,o.jsx)(n.strong,{children:"Consistent Naming Conventions"}),": Use meaningful and consistent naming conventions to easily identify and manage your nodes."]}),"\n",(0,o.jsxs)(n.li,{children:[(0,o.jsx)(n.strong,{children:"Dynamic Configuration"}),": Take advantage of ROS 2\u2019s dynamic remapping capabilities to flexibly configure nodes at runtime."]}),"\n",(0,o.jsxs)(n.li,{children:[(0,o.jsx)(n.strong,{children:"Testing and Verification"}),": Regularly verify the configurations and interactions of your nodes to ensure correct operation."]}),"\n"]}),"\n",(0,o.jsx)(n.h2,{id:"conclusion",children:"Conclusion"}),"\n",(0,o.jsx)(n.p,{children:"Launching multiple instances of a node with different configurations in ROS 2 is a powerful capability that can streamline many robotic applications. By following the best practices outlined in this tutorial, you can ensure robust and error-free operation of your nodes. Proper node management and dynamic configuration are key to leveraging the full potential of ROS 2 in complex systems."})]})}function u(e={}){const{wrapper:n}={...(0,t.R)(),...e.components};return n?(0,o.jsx)(n,{...e,children:(0,o.jsx)(d,{...e})}):d(e)}},8453:(e,n,i)=>{i.d(n,{R:()=>a,x:()=>r});var s=i(6540);const o={},t=s.createContext(o);function a(e){const n=s.useContext(t);return s.useMemo((function(){return"function"==typeof e?e(n):{...n,...e}}),[n,e])}function r(e){let n;return n=e.disableParentContext?"function"==typeof e.components?e.components(o):e.components||o:a(e.components),s.createElement(t.Provider,{value:n},e.children)}}}]);