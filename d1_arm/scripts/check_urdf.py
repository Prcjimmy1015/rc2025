from ikpy.chain import Chain
chain = Chain.from_urdf_file("d1_description.urdf")
for i, link in enumerate(chain.links):
    print(f"Link {i}: {link.name}, active={link.active}, type={type(link).__name__}")
