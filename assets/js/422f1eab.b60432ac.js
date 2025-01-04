"use strict";(self.webpackChunkmy_website=self.webpackChunkmy_website||[]).push([[5222],{2350:(e,n,t)=>{t.r(n),t.d(n,{assets:()=>l,contentTitle:()=>c,default:()=>h,frontMatter:()=>a,metadata:()=>i,toc:()=>o});const i=JSON.parse('{"id":"interfaces/create-build-custom-srv","title":"Creating and Building a Custom ROS2 Service","description":"In this tutorial, we will provide a detailed guide on how to create and build a custom service in ROS2. This process assumes you have some familiarity with creating and building messages in ROS2, as the steps for services are quite similar.","source":"@site/docs/05_interfaces/05_create-build-custom-srv.md","sourceDirName":"05_interfaces","slug":"/interfaces/create-build-custom-srv","permalink":"/ROS2-Beginners/docs/interfaces/create-build-custom-srv","draft":false,"unlisted":false,"editUrl":"https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/docs/05_interfaces/05_create-build-custom-srv.md","tags":[],"version":"current","sidebarPosition":5,"frontMatter":{},"sidebar":"tutorialSidebar","previous":{"title":"Creating and Using Custom Messages in ROS2 with Python","permalink":"/ROS2-Beginners/docs/interfaces/use-custom-msg-py"},"next":{"title":"ROS 2 Interface Management and Usage Tutorial","permalink":"/ROS2-Beginners/docs/interfaces/debug-msg-srv-ros2-tools"}}');var s=t(4848),r=t(8453);const a={},c="Creating and Building a Custom ROS2 Service",l={},o=[{value:"Step 1: Set Up Your Package",id:"step-1-set-up-your-package",level:2},{value:"Step 2: Create the Service Definition File",id:"step-2-create-the-service-definition-file",level:2},{value:"Step 3: Update <code>CMakeLists.txt</code>",id:"step-3-update-cmakeliststxt",level:2},{value:"Step 4: Update <code>package.xml</code>",id:"step-4-update-packagexml",level:2},{value:"Step 5: Build the Package",id:"step-5-build-the-package",level:2},{value:"Step 6: Verify the Service",id:"step-6-verify-the-service",level:2},{value:"Step 7: Implementing the Service in Python",id:"step-7-implementing-the-service-in-python",level:2},{value:"Step 8: Implementing the Client in Python",id:"step-8-implementing-the-client-in-python",level:2},{value:"Step 9: Running the Service and Client",id:"step-9-running-the-service-and-client",level:2},{value:"Conclusion",id:"conclusion",level:2}];function d(e){const n={code:"code",h1:"h1",h2:"h2",header:"header",p:"p",pre:"pre",...(0,r.R)(),...e.components};return(0,s.jsxs)(s.Fragment,{children:[(0,s.jsx)(n.header,{children:(0,s.jsx)(n.h1,{id:"creating-and-building-a-custom-ros2-service",children:"Creating and Building a Custom ROS2 Service"})}),"\n",(0,s.jsx)(n.p,{children:"In this tutorial, we will provide a detailed guide on how to create and build a custom service in ROS2. This process assumes you have some familiarity with creating and building messages in ROS2, as the steps for services are quite similar."}),"\n",(0,s.jsx)(n.h2,{id:"step-1-set-up-your-package",children:"Step 1: Set Up Your Package"}),"\n",(0,s.jsxs)(n.p,{children:["First, we will use the existing package ",(0,s.jsx)(n.code,{children:"my_robot_interfaces"}),". Ensure you have this package created and properly set up."]}),"\n",(0,s.jsx)(n.h2,{id:"step-2-create-the-service-definition-file",children:"Step 2: Create the Service Definition File"}),"\n",(0,s.jsxs)(n.p,{children:["Navigate to the ",(0,s.jsx)(n.code,{children:"msg"})," folder within your package and create a new subfolder named ",(0,s.jsx)(n.code,{children:"srv"}),". Inside this ",(0,s.jsx)(n.code,{children:"srv"})," folder, we will define our custom service."]}),"\n",(0,s.jsxs)(n.p,{children:["Create a file named ",(0,s.jsx)(n.code,{children:"ComputeRectangleArea.srv"})," with the following content:"]}),"\n",(0,s.jsx)(n.pre,{children:(0,s.jsx)(n.code,{className:"language-plaintext",children:"# Request part\nfloat64 length\nfloat64 width\n---\n# Response part\nfloat64 area\n"})}),"\n",(0,s.jsx)(n.p,{children:"This service will take the length and width of a rectangle as input and return the area as output."}),"\n",(0,s.jsxs)(n.h2,{id:"step-3-update-cmakeliststxt",children:["Step 3: Update ",(0,s.jsx)(n.code,{children:"CMakeLists.txt"})]}),"\n",(0,s.jsxs)(n.p,{children:["Ensure that your ",(0,s.jsx)(n.code,{children:"CMakeLists.txt"})," file is configured to process the service definitions. Add the service file to the ",(0,s.jsx)(n.code,{children:"rosidl_generate_interfaces"})," section. Your ",(0,s.jsx)(n.code,{children:"CMakeLists.txt"})," should include:"]}),"\n",(0,s.jsx)(n.pre,{children:(0,s.jsx)(n.code,{className:"language-cmake",children:'find_package(rosidl_default_generators REQUIRED)\n\nrosidl_generate_interfaces(${PROJECT_NAME}\n  "srv/ComputeRectangleArea.srv"\n)\n'})}),"\n",(0,s.jsxs)(n.h2,{id:"step-4-update-packagexml",children:["Step 4: Update ",(0,s.jsx)(n.code,{children:"package.xml"})]}),"\n",(0,s.jsxs)(n.p,{children:["Make sure your ",(0,s.jsx)(n.code,{children:"package.xml"})," includes the necessary dependencies. Add the following lines if they are not already present:"]}),"\n",(0,s.jsx)(n.pre,{children:(0,s.jsx)(n.code,{className:"language-xml",children:"<build_depend>rosidl_default_generators</build_depend>\n<exec_depend>rosidl_default_runtime</exec_depend>\n"})}),"\n",(0,s.jsx)(n.h2,{id:"step-5-build-the-package",children:"Step 5: Build the Package"}),"\n",(0,s.jsxs)(n.p,{children:["With the service definition in place and the necessary configurations in your ",(0,s.jsx)(n.code,{children:"CMakeLists.txt"})," and ",(0,s.jsx)(n.code,{children:"package.xml"}),", you can now build your package. Use the following command to build the package:"]}),"\n",(0,s.jsx)(n.pre,{children:(0,s.jsx)(n.code,{className:"language-bash",children:"colcon build --packages-select my_robot_interfaces\n"})}),"\n",(0,s.jsx)(n.p,{children:"After building, source the setup file:"}),"\n",(0,s.jsx)(n.pre,{children:(0,s.jsx)(n.code,{className:"language-bash",children:"source install/setup.bash\n"})}),"\n",(0,s.jsx)(n.h2,{id:"step-6-verify-the-service",children:"Step 6: Verify the Service"}),"\n",(0,s.jsxs)(n.p,{children:["You can verify that your service has been created correctly by using the ",(0,s.jsx)(n.code,{children:"ros2 interface show"})," command:"]}),"\n",(0,s.jsx)(n.pre,{children:(0,s.jsx)(n.code,{className:"language-bash",children:"ros2 interface show my_robot_interfaces/srv/ComputeRectangleArea\n"})}),"\n",(0,s.jsx)(n.p,{children:"This command should display the definition of your service."}),"\n",(0,s.jsx)(n.h2,{id:"step-7-implementing-the-service-in-python",children:"Step 7: Implementing the Service in Python"}),"\n",(0,s.jsx)(n.p,{children:"Next, we will create a Python service node to handle requests for calculating the area of a rectangle."}),"\n",(0,s.jsxs)(n.p,{children:["Create a new Python script named ",(0,s.jsx)(n.code,{children:"rectangle_area_service.py"})," in the ",(0,s.jsx)(n.code,{children:"my_robot_interfaces"})," package's ",(0,s.jsx)(n.code,{children:"src"})," directory with the following content:"]}),"\n",(0,s.jsx)(n.pre,{children:(0,s.jsx)(n.code,{className:"language-python",children:"import rclpy\nfrom rclpy.node import Node\nfrom my_robot_interfaces.srv import ComputeRectangleArea\n\nclass RectangleAreaService(Node):\n\n    def __init__(self):\n        super().__init__('rectangle_area_service')\n        self.srv = self.create_service(ComputeRectangleArea, 'compute_rectangle_area', self.compute_rectangle_area_callback)\n\n    def compute_rectangle_area_callback(self, request, response):\n        response.area = request.length * request.width\n        self.get_logger().info(f'Request: length={request.length}, width={request.width}, area={response.area}')\n        return response\n\ndef main(args=None):\n    rclpy.init(args=args)\n    rectangle_area_service = RectangleAreaService()\n    rclpy.spin(rectangle_area_service)\n    rclpy.shutdown()\n\nif __name__ == '__main__':\n    main()\n"})}),"\n",(0,s.jsx)(n.h2,{id:"step-8-implementing-the-client-in-python",children:"Step 8: Implementing the Client in Python"}),"\n",(0,s.jsxs)(n.p,{children:["Similarly, create a client node to send requests to the service. Create a file named ",(0,s.jsx)(n.code,{children:"rectangle_area_client.py"})," with the following content:"]}),"\n",(0,s.jsx)(n.pre,{children:(0,s.jsx)(n.code,{className:"language-python",children:"import sys\nimport rclpy\nfrom rclpy.node import Node\nfrom my_robot_interfaces.srv import ComputeRectangleArea\n\nclass RectangleAreaClient(Node):\n\n    def __init__(self):\n        super().__init__('rectangle_area_client')\n        self.client = self.create_client(ComputeRectangleArea, 'compute_rectangle_area')\n        while not self.client.wait_for_service(timeout_sec=1.0):\n            self.get_logger().info('Service not available, waiting...')\n        self.request = ComputeRectangleArea.Request()\n\n    def send_request(self):\n        self.request.length = float(sys.argv[1])\n        self.request.width = float(sys.argv[2])\n        future = self.client.call_async(self.request)\n        rclpy.spin_until_future_complete(self, future)\n        return future.result()\n\ndef main(args=None):\n    rclpy.init(args=args)\n    rectangle_area_client = RectangleAreaClient()\n    response = rectangle_area_client.send_request()\n    rectangle_area_client.get_logger().info(f'Result: {response.area}')\n    rclpy.shutdown()\n\nif __name__ == '__main__':\n    main()\n"})}),"\n",(0,s.jsx)(n.h2,{id:"step-9-running-the-service-and-client",children:"Step 9: Running the Service and Client"}),"\n",(0,s.jsx)(n.p,{children:"To run the service node, use the following command:"}),"\n",(0,s.jsx)(n.pre,{children:(0,s.jsx)(n.code,{className:"language-bash",children:"ros2 run my_robot_interfaces rectangle_area_service\n"})}),"\n",(0,s.jsx)(n.p,{children:"In a new terminal, run the client node with the length and width as arguments:"}),"\n",(0,s.jsx)(n.pre,{children:(0,s.jsx)(n.code,{className:"language-bash",children:"ros2 run my_robot_interfaces rectangle_area_client 5.0 3.0\n"})}),"\n",(0,s.jsx)(n.p,{children:"The client will send the length and width to the service, and the service will respond with the area of the rectangle."}),"\n",(0,s.jsx)(n.h2,{id:"conclusion",children:"Conclusion"}),"\n",(0,s.jsx)(n.p,{children:"In this tutorial, we created a custom service in ROS2, defined its message structure, updated the package configuration, built the package, and implemented both a service and a client in Python. This provides a solid foundation for creating and using custom services in your ROS2 projects."})]})}function h(e={}){const{wrapper:n}={...(0,r.R)(),...e.components};return n?(0,s.jsx)(n,{...e,children:(0,s.jsx)(d,{...e})}):d(e)}},8453:(e,n,t)=>{t.d(n,{R:()=>a,x:()=>c});var i=t(6540);const s={},r=i.createContext(s);function a(e){const n=i.useContext(r);return i.useMemo((function(){return"function"==typeof e?e(n):{...n,...e}}),[n,e])}function c(e){let n;return n=e.disableParentContext?"function"==typeof e.components?e.components(s):e.components||s:a(e.components),i.createElement(r.Provider,{value:n},e.children)}}}]);