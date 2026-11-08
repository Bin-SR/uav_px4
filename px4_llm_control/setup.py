from setuptools import setup

setup(
    name='px4_llm_control', version='0.1.0', packages=['px4_llm_control'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/px4_llm_control']),
                ('share/px4_llm_control', ['package.xml', 'README.md'])],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='si', maintainer_email='1072235132@qq.com', license='Apache-2.0',
    description='DeepSeek natural language control for PX4 SITL',
    entry_points={'console_scripts': [
        'llm_control = px4_llm_control.node:main']},
)
