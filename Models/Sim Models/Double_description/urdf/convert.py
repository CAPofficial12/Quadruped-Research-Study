import xacro

input_file = "../Double.urdf"
output_file = "Double.urdf"

doc = xacro.process_file(input_file)

with open(output_file, "w", encoding="utf-8") as f:
    f.write(doc.toxml())

print(f"Created {output_file}")