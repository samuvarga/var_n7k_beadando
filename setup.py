from setuptools import setup

package_name = 'var_n7k_beadando'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'var_n7k_beadando.sierpinski_triangle_node'
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='he',
    maintainer_email='samuvarga24@gmail.com',
    description='TODO: Package description',
    license='GNU General Public License v3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sierpinski_triangle_node = var_n7k_beadando.sierpinski_triangle_node:main'
        ],
    },
)
