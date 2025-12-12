from xacrodoc import XacroDoc

# Load the Xacro file
doc = XacroDoc.from_file(r"C:\Users\Thane\Downloads\urdf_export_test\full_body\full_body_description\urdf\full_body.xacro")

# Convert to a string of URDF
urdf_str = doc.to_urdf_string()

# Write to a file
doc.to_urdf_file(r"C:\Users\Thane\Downloads\urdf_export_test\full_body\full_body_description\urdf\full_body.urdf")