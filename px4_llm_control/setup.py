from setuptools import setup

setup(
    name='px4_llm_control', version='0.3.0', packages=['px4_llm_control'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/px4_llm_control']),
                ('share/px4_llm_control', ['package.xml', 'README.md', 'MULTIMODAL.md', 'GAZEBO_PLAN.md']),
                ('share/px4_llm_control/examples', ['examples/square_plan.json']),
                ('share/px4_llm_control/config', ['config/multimodal.yaml'])],
    install_requires=['setuptools', 'numpy', 'Pillow'], zip_safe=True,
    maintainer='si', maintainer_email='1072235132@qq.com', license='Apache-2.0',
    description='DeepSeek natural language control for PX4 SITL',
    entry_points={'console_scripts': [
        'llm_control = px4_llm_control.node:main',
        'multimodal_model_control = px4_llm_control.multimodal_model_control:main']},
)
