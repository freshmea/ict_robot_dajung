# from test_package import package_a as a
# from test_package import package_b as b
from test_package import *

def main():
    print(package_a.variable_a)
    package_a.a_test()

    print(package_b.variable_b)
    package_b.b_test()

if __name__ == '__main__':
    main()