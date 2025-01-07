"use strict";(self.webpackChunkROS2_Beginners=self.webpackChunkROS2_Beginners||[]).push([[6146],{3462:(e,n,i)=>{i.r(n),i.d(n,{assets:()=>t,contentTitle:()=>r,default:()=>h,frontMatter:()=>c,metadata:()=>s,toc:()=>d});const s=JSON.parse('{"id":"tools/colcon","title":"Workspace Building and Efficient Development with Python Nodes","description":"This tutorial provides an in-depth understanding of building and managing a ROS 2 workspace, with a specific focus on optimizing the workflow for Python nodes. We will critically address and refine the concepts presented, ensuring accuracy and adherence to professional standards.","source":"@site/docs/02_tools/04_colcon.md","sourceDirName":"02_tools","slug":"/tools/colcon","permalink":"/ROS2-Beginners/docs/tools/colcon","draft":false,"unlisted":false,"editUrl":"https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/docs/02_tools/04_colcon.md","tags":[],"version":"current","sidebarPosition":4,"frontMatter":{},"sidebar":"tutorialSidebar","previous":{"title":"Launching Multiple Instances of a Node with Different Configurations","permalink":"/ROS2-Beginners/docs/tools/rename-node-runtime"},"next":{"title":"rqt and rqt_graph in ROS 2","permalink":"/ROS2-Beginners/docs/tools/rqt-rqt_graph"}}');var o=i(4848),l=i(8453);const c={},r="Workspace Building and Efficient Development with Python Nodes",t={},d=[{value:"Building a ROS 2 Workspace",id:"building-a-ros-2-workspace",level:2},{value:"Setting Up Your Workspace",id:"setting-up-your-workspace",level:2},{value:"Building the Workspace",id:"building-the-workspace",level:2},{value:"Building Specific Packages",id:"building-specific-packages",level:2},{value:"Auto-completion for <code>colcon</code>",id:"auto-completion-for-colcon",level:2},{value:"Efficient Python Node Development with Symlink Install",id:"efficient-python-node-development-with-symlink-install",level:2},{value:"Standard Build vs. Symlink Install",id:"standard-build-vs-symlink-install",level:2},{value:"Step-by-Step Example",id:"step-by-step-example",level:2},{value:"Important Considerations",id:"important-considerations",level:2}];function a(e){const n={code:"code",h1:"h1",h2:"h2",header:"header",li:"li",ol:"ol",p:"p",pre:"pre",strong:"strong",ul:"ul",...(0,l.R)(),...e.components};return(0,o.jsxs)(o.Fragment,{children:[(0,o.jsx)(n.header,{children:(0,o.jsx)(n.h1,{id:"workspace-building-and-efficient-development-with-python-nodes",children:"Workspace Building and Efficient Development with Python Nodes"})}),"\n",(0,o.jsx)(n.p,{children:"This tutorial provides an in-depth understanding of building and managing a ROS 2 workspace, with a specific focus on optimizing the workflow for Python nodes. We will critically address and refine the concepts presented, ensuring accuracy and adherence to professional standards."}),"\n",(0,o.jsx)(n.h2,{id:"building-a-ros-2-workspace",children:"Building a ROS 2 Workspace"}),"\n",(0,o.jsxs)(n.p,{children:["ROS 2 (Robot Operating System 2) is an open-source framework for building robot applications. It uses ",(0,o.jsx)(n.code,{children:"colcon"})," as its build tool, which in turn uses ",(0,o.jsx)(n.code,{children:"ament"})," as the build system. This section will detail how to set up and build a ROS 2 workspace efficiently."]}),"\n",(0,o.jsx)(n.h2,{id:"setting-up-your-workspace",children:"Setting Up Your Workspace"}),"\n",(0,o.jsxs)(n.ol,{children:["\n",(0,o.jsxs)(n.li,{children:["\n",(0,o.jsxs)(n.p,{children:[(0,o.jsx)(n.strong,{children:"Create a Workspace Directory"}),":"]}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{children:"mkdir -p ~/ros2_ws/src\ncd ~/ros2_ws\n"})}),"\n"]}),"\n",(0,o.jsxs)(n.li,{children:["\n",(0,o.jsxs)(n.p,{children:[(0,o.jsxs)(n.strong,{children:["Add Packages to the ",(0,o.jsx)(n.code,{children:"src"})," Directory"]}),":\nPlace your packages in the ",(0,o.jsx)(n.code,{children:"src"})," directory. This could include both Python and C++ packages."]}),"\n"]}),"\n"]}),"\n",(0,o.jsx)(n.h2,{id:"building-the-workspace",children:"Building the Workspace"}),"\n",(0,o.jsx)(n.p,{children:"To build the entire workspace:"}),"\n",(0,o.jsxs)(n.ol,{children:["\n",(0,o.jsxs)(n.li,{children:["\n",(0,o.jsxs)(n.p,{children:[(0,o.jsx)(n.strong,{children:"Navigate to the Workspace Root"}),":"]}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{children:"cd ~/ros2_ws\n"})}),"\n"]}),"\n",(0,o.jsxs)(n.li,{children:["\n",(0,o.jsxs)(n.p,{children:[(0,o.jsxs)(n.strong,{children:["Run ",(0,o.jsx)(n.code,{children:"colcon build"})]}),":"]}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{children:"colcon build\n"})}),"\n",(0,o.jsxs)(n.p,{children:["This command builds all packages located in the ",(0,o.jsx)(n.code,{children:"src"})," directory."]}),"\n"]}),"\n"]}),"\n",(0,o.jsx)(n.h2,{id:"building-specific-packages",children:"Building Specific Packages"}),"\n",(0,o.jsxs)(n.p,{children:["To build specific packages, use the ",(0,o.jsx)(n.code,{children:"--packages-select"})," option:"]}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{children:"colcon build --packages-select <package_name>\n"})}),"\n",(0,o.jsxs)(n.p,{children:["For example, to build only ",(0,o.jsx)(n.code,{children:"my_python_package"}),":"]}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{children:"colcon build --packages-select my_python_package\n"})}),"\n",(0,o.jsxs)(n.h2,{id:"auto-completion-for-colcon",children:["Auto-completion for ",(0,o.jsx)(n.code,{children:"colcon"})]}),"\n",(0,o.jsxs)(n.p,{children:["Auto-completion can significantly speed up your workflow. Ensure the following line is added to your ",(0,o.jsx)(n.code,{children:".bashrc"})," file:"]}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{children:"source /usr/share/colcon_cd/function/colcon_cd.sh\n"})}),"\n",(0,o.jsxs)(n.p,{children:["After adding this line, reload your ",(0,o.jsx)(n.code,{children:".bashrc"}),":"]}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{children:"source ~/.bashrc\n"})}),"\n",(0,o.jsx)(n.h2,{id:"efficient-python-node-development-with-symlink-install",children:"Efficient Python Node Development with Symlink Install"}),"\n",(0,o.jsxs)(n.p,{children:["Developing with Python nodes in ROS 2 can be streamlined by using the ",(0,o.jsx)(n.code,{children:"--symlink-install"})," flag. This flag creates symbolic links to your Python files, allowing changes to be reflected immediately without the need for rebuilding."]}),"\n",(0,o.jsx)(n.h2,{id:"standard-build-vs-symlink-install",children:"Standard Build vs. Symlink Install"}),"\n",(0,o.jsxs)(n.ol,{children:["\n",(0,o.jsxs)(n.li,{children:["\n",(0,o.jsxs)(n.p,{children:[(0,o.jsx)(n.strong,{children:"Standard Build"}),":"]}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{children:"colcon build\n"})}),"\n",(0,o.jsx)(n.p,{children:"Each modification to your Python node requires a rebuild:"}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{children:"colcon build --packages-select my_python_package\n"})}),"\n"]}),"\n",(0,o.jsxs)(n.li,{children:["\n",(0,o.jsxs)(n.p,{children:[(0,o.jsx)(n.strong,{children:"Using Symlink Install"}),":"]}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{children:"colcon build --symlink-install\n"})}),"\n",(0,o.jsx)(n.p,{children:"This method eliminates the need for repeated builds. The symbolic links ensure that any changes in your Python files are immediately effective."}),"\n"]}),"\n"]}),"\n",(0,o.jsx)(n.h2,{id:"step-by-step-example",children:"Step-by-Step Example"}),"\n",(0,o.jsxs)(n.ol,{children:["\n",(0,o.jsxs)(n.li,{children:["\n",(0,o.jsxs)(n.p,{children:[(0,o.jsx)(n.strong,{children:"Create a Python Node"}),":"]}),"\n",(0,o.jsxs)(n.ul,{children:["\n",(0,o.jsxs)(n.li,{children:["\n",(0,o.jsx)(n.p,{children:"Navigate to your package directory:"}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{children:"cd ~/ros2_ws/src/my_python_package\n"})}),"\n"]}),"\n",(0,o.jsxs)(n.li,{children:["\n",(0,o.jsxs)(n.p,{children:["Ensure your Python file (",(0,o.jsx)(n.code,{children:"my_first_node.py"}),") is executable:"]}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{children:"chmod +x my_first_node.py\n"})}),"\n"]}),"\n"]}),"\n"]}),"\n",(0,o.jsxs)(n.li,{children:["\n",(0,o.jsxs)(n.p,{children:[(0,o.jsx)(n.strong,{children:"Build the Package with Symlink Install"}),":"]}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{children:"colcon build --packages-select my_python_package --symlink-install\n"})}),"\n"]}),"\n",(0,o.jsxs)(n.li,{children:["\n",(0,o.jsxs)(n.p,{children:[(0,o.jsx)(n.strong,{children:"Running the Node"}),":"]}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{children:"ros2 run my_python_package my_first_node\n"})}),"\n"]}),"\n",(0,o.jsxs)(n.li,{children:["\n",(0,o.jsxs)(n.p,{children:[(0,o.jsx)(n.strong,{children:"Modifying and Testing Without Rebuilding"}),":"]}),"\n",(0,o.jsxs)(n.ul,{children:["\n",(0,o.jsxs)(n.li,{children:["\n",(0,o.jsxs)(n.p,{children:["Make changes to ",(0,o.jsx)(n.code,{children:"my_first_node.py"}),"."]}),"\n"]}),"\n",(0,o.jsxs)(n.li,{children:["\n",(0,o.jsx)(n.p,{children:"Run the node again without rebuilding:"}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{children:"ros2 run my_python_package my_first_node\n"})}),"\n"]}),"\n"]}),"\n"]}),"\n"]}),"\n",(0,o.jsx)(n.h2,{id:"important-considerations",children:"Important Considerations"}),"\n",(0,o.jsxs)(n.ul,{children:["\n",(0,o.jsxs)(n.li,{children:[(0,o.jsx)(n.strong,{children:"Executable Python File"}),": Ensure your Python script is marked as executable. Failure to do so will result in errors when using symlink installs."]}),"\n",(0,o.jsxs)(n.li,{children:[(0,o.jsx)(n.strong,{children:"Initial Compilation"}),": The initial build is necessary to set up the environment and generate necessary files. Subsequent modifications can leverage the symlink install for efficiency."]}),"\n",(0,o.jsxs)(n.li,{children:[(0,o.jsx)(n.strong,{children:"C++ Nodes"}),": The symlink install method is not applicable to C++ nodes, which require compilation after each modification."]}),"\n"]}),"\n",(0,o.jsx)(n.p,{children:"By following these detailed instructions, you can optimize your ROS 2 workspace for efficient development, particularly when working with Python nodes. This approach minimizes the time spent on repetitive builds, allowing you to focus on developing and testing your robotic applications more effectively."})]})}function h(e={}){const{wrapper:n}={...(0,l.R)(),...e.components};return n?(0,o.jsx)(n,{...e,children:(0,o.jsx)(a,{...e})}):a(e)}},8453:(e,n,i)=>{i.d(n,{R:()=>c,x:()=>r});var s=i(6540);const o={},l=s.createContext(o);function c(e){const n=s.useContext(l);return s.useMemo((function(){return"function"==typeof e?e(n):{...n,...e}}),[n,e])}function r(e){let n;return n=e.disableParentContext?"function"==typeof e.components?e.components(o):e.components||o:c(e.components),s.createElement(l.Provider,{value:n},e.children)}}}]);