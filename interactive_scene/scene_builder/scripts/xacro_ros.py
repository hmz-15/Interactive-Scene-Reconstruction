from lxml import etree as ET
from lxml.builder import ElementMaker



EM = ElementMaker(
    namespace="http://ros.org/wiki/xacro",
    nsmap={"xacro": "http://ros.org/wiki/xacro"}
)


class XacroROS(object):
    
    def __init__(self, root_name):
        self.root_ = ET.Element("robot", nsmap={"xacro": "http://ros.org/wiki/xacro"})
        
        # set root attributes
        self.root_.set("name", root_name)

        self.macros_ = {}
        

    def __str__(self):
        return ET.tostring(self.root_, pretty_print=True).decode()

    
    def save(self, filename):
        """
        Write current xacro into a file

        @param filename: (string) the output directory
        """
        tree = ET.ElementTree(self.root_)
        tree.write(filename, pretty_print=True, xml_declaration=True, encoding="utf-8")

        print("[INFO] xacro file is saved at: {}".format(filename))


    def add_macro(self, macro_name):
        """
        Define a new macro inside the xacro

        @param macro_name: (string) the name of the macro
        """
        macro = EM.macro()
        macro.set("name", macro_name)

        self.root_.append(macro)
        self.macros_[macro_name] = macro


    def add_include(self, inc_dir):
        """
        Add a include command into the xacro scene

        Include another xacro file before instantiate the macro defined in that file

        @param inc_dir: (string) the directory to be included 
        """
        inc = EM.include()
        inc.set("filename", inc_dir)

        self.root_.append(inc)


    def add_background(self, link_name, mesh_dir, xyz, rpy, 
        scale=[1.0, 1.0, 1.0], mass=1, inertia=[1, 0, 0, 1, 0, 1], add_collision=False, ns=None
    ):
        """
        Add a background object into xacro

        A background object only has visual element

        @param link_name: (string) the link name of the new added object
        @param mesh_dir: (string) directory of the object mesh model
        @param xyz: (list) a list of [x, y, z] that specifies the position of the origin tf
        @param rpy: (list) a list of [roll, pitch, yaw] that specifies the orientation of the origin tf
        @param scale: (list) a list of [x, y, z] that specify the scale along each axis
        @param mass: (float) the mass of the link
        @param inertia (list): a list of floats [ixx, ixy, ixz, iyy, iyz, izz] that encodes the inertia
        @param add_collision (boolean): if enable link collision property 
        @param ns: (string) under which namespace, the object to be added
        """
        parent = self.get_parent_element_(ns)
        
        link = ET.SubElement(parent, "link")
        link.set("name", link_name)

        visual_et = ET.SubElement(link, "visual")
        self.add_mesh_to_element_(visual_et, mesh_dir, xyz, rpy, scale)

        if add_collision:
            collision_et = ET.SubElement(link, "collision")
            self.add_mesh_to_element_(collision_et, mesh_dir, xyz, rpy, scale)
        
        if (mass is not None) or (inertia is not None):
            self.add_interial_to_element_(link, xyz, rpy, mass, inertia)


    def add_object(self, link_name, mesh_dir, xyz, rpy, 
        scale=[1.0, 1.0, 1.0], mass=1, inertia=[1, 0, 0, 1, 0, 1], add_collision=False, ns=None
    ):
        """
        Add new object into xacro

        @param link_name: (string) the link name of the new added object
        @param mesh_dir: (string) directory of the object mesh model
        @param xyz: (list) a list of [x, y, z] that specifies the position of the origin tf
        @param rpy: (list) a list of [roll, pitch, yaw] that specifies the orientation of the origin tf
        @param scale: (list) a list of [x, y, z] that specify the scale along each axis
        @param mass: (float) the mass of the link
        @param inertia (list): a list of floats [ixx, ixy, ixz, iyy, iyz, izz] that encodes the inertia
        @param add_collision (boolean): if enable link collision property 
        @param ns: (string) under which namespace, the object to be added
        """
        parent = self.get_parent_element_(ns)
        
        link = ET.SubElement(parent, "link")
        link.set("name", link_name)

        visual = ET.SubElement(link, "visual")
        self.add_mesh_to_element_(visual, mesh_dir, xyz, rpy, scale)

        if add_collision:
            collision_et = ET.SubElement(link, "collision")
            self.add_mesh_to_element_(collision_et, mesh_dir, xyz, rpy, scale)
        
        if (mass is not None) or (inertia is not None):
            self.add_interial_to_element_(link, xyz, rpy, mass, inertia)


    def add_link(self, link_name, gazebo_dummy_link=False, ns=None):
        """
        Add a single link into the xacro scene

        @param link_name: (string) the link name of the new added object
        @param ns: (string) under which namespace, the object to be added
        """
        parent = self.get_parent_element_(ns)

        link = ET.SubElement(parent, "link")
        link.set("name", link_name)
        
        if gazebo_dummy_link:
            self.add_gazebo_dummy_attr_(link)


    def add_joint(self, joint_name, joint_type, child_link, parent_link, 
        xyz, rpy, add_gazebo_tag=False, ns=None
    ):
        """
        Add a joint (transform) between child link and parent link

        @param joint_name: (string) name of the joint
        @param child_link: (string): name of the child link
        @param parent_link: (string): name of the parent link
        @param xyz (list): position <x, y, z> of the transform
        @param rpy (list): euler angle <roll, pitch, yaw> of the transform
        @param add_gazebo_tag (boolean): add gazebo tag for the joint
            (i.e., disableFixedJointLumping, preserveFixedJoint)
        """
        parent = self.get_parent_element_(ns)

        joint = ET.SubElement(parent, "joint")
        joint.set("name", joint_name)
        joint.set("type", joint_type)

        origin = ET.SubElement(joint, "origin")
        origin.set("xyz", "{} {} {}".format(*xyz))
        origin.set("rpy", "{} {} {}".format(*rpy))

        plink_element = ET.SubElement(joint, "parent")
        plink_element.set("link", parent_link)

        clink_element = ET.SubElement(joint, "child")
        clink_element.set("link", child_link)

        if add_gazebo_tag:
            # preserve fixed joint in Gazebo
            if joint_type == "fixed":
                self.add_gazebo_pfj_tag_(parent, joint_name)

    
    def instantiate_macro(self, macro_name, **kwargs):
        """
        Instantiate a pre-defined macro
        """
        code_line = "EM.{}()".format(macro_name)
        
        # This will assign ins_macro with the expression code_line
        # eval() runs the string as an python expression
        ins_macro = eval(code_line)

        for k, v in kwargs.items():
            ins_macro.set(k, v)

        self.root_.append(ins_macro)


    def get_parent_element_(self, ns):
        """
        Add new object into xacro

        If ns is None, then the root element will be returned

        @param ns: (string) namespace
        """
        if ns is None:
            return self.root_
        else:
            assert(ns in self.macros_)
            return self.macros_[ns]


    def add_mesh_to_element_(self, element, mesh_dir, xyz, rpy, scale=[1.0, 1.0, 1.0]):
        """
        Add a mesh objects under the given element

        @param element: (ET.Element) the target element
        @param mesh_dir: (string): the directory of the mesh file
        @param xyz (list): position <x, y, z> of the transform
        @param rpy (list): euler angle <roll, pitch, yaw> of the transform
        @param scale (list): euler angle <roll, pitch, yaw> of the transform
        """
        origin = ET.SubElement(element, "origin")
        origin.set("xyz", "{} {} {}".format(*xyz))
        origin.set("rpy", "{} {} {}".format(*rpy))

        geom = ET.SubElement(element, "geometry")
        
        mesh = ET.SubElement(geom, "mesh")
        mesh.set("filename", mesh_dir)
        mesh.set("scale", "{} {} {}".format(*scale))


    def add_interial_to_element_(self, element, xyz, rpy, mass, inertia):
        """
        Add the inertia info under the given element

        @param element: (ET.Element) the target element
        @param xyz (list): position <x, y, z> of the transform
        @param rpy (list): euler angle <roll, pitch, yaw> of the transform
        @param mass (float): the mass of the link
        @param inertia (list): a list of floats [ixx, ixy, ixz, iyy, iyz, izz] that encodes the inertia
        """
        assert(mass is not None)
        assert(inertia is not None)

        inertial_et = ET.SubElement(element, "inertial")
        
        origin_et = ET.SubElement(inertial_et, "origin")
        origin_et.set("xyz", "{} {} {}".format(*xyz))
        origin_et.set("rpy", "{} {} {}".format(*rpy))

        mass_et = ET.SubElement(inertial_et, "mass")
        mass_et.set("value", "{}".format(mass))

        inertia_et = ET.SubElement(inertial_et, "inertia")
        inertia_attr = ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"]
        for attr, val in zip(inertia_attr, inertia):
            inertia_et.set(attr, "{}".format(val))


    def add_gazebo_pfj_tag_(self, element, joint_name):
        """
        Add gazebo tag to preserve the fixed joint

        @param element: (ET.Element) the target element
        @param joint_name: (string) reference joint name
        """
        gazebo_et = ET.SubElement(element, "gazebo")
        gazebo_et.set("reference", joint_name)

        tag = ET.SubElement(gazebo_et, "disableFixedJointLumping")
        tag.text = "true"

        tag = ET.SubElement(gazebo_et, "preserveFixedJoint")
        tag.text = "true"
    

    def add_gazebo_dummy_attr_(self, element):
        """
        Add gazebo dummy attributes to current element
            visual
            collision
            inertial

        @param element: (ET.Element) the target element
        """
        visual_et = ET.SubElement(element, "visual")
        self.add_box_marker_(visual_et, xyz=[0, 0, 0], rpy=[0, 0, 0], size=[0.01, 0.01, 0.01])

        collision_et = ET.SubElement(element, "collision")
        self.add_box_marker_(collision_et, xyz=[0, 0, 0], rpy=[0, 0, 0], size=[0, 0, 0])
        
        self.add_interial_to_element_(element, xyz=[0, 0, 0], rpy=[0, 0, 0], mass=1, inertia=[1, 0, 0, 1, 0, 1])
    

    def add_box_marker_(self, element, xyz, rpy, size=[1, 1, 1]):
        """
        Add a box marker to current element

        @param element: (ET.Element) the target element
        @param xyz (list): position <x, y, z> of the transform
        @param rpy (list): euler angle <roll, pitch, yaw> of the transform
        @param size (list): dimension of the box (x, y, z)
        """
        origin_et = ET.SubElement(element, "origin")
        origin_et.set("xyz", "{} {} {}".format(*xyz))
        origin_et.set("rpy", "{} {} {}".format(*rpy))

        geom_et = ET.SubElement(element, "geometry")
        
        box_et = ET.SubElement(geom_et, "box")
        box_et.set("size", "{} {} {}".format(*size))


def test_xacro_ros():
    xacro = XacroROS("world")
    xacro.add_macro("create_world")
    xacro.add_object("create_world")
    print(xacro)


if __name__ == "__main__":
    test_xacro_ros()