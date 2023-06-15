import test_module as test

def main():
    radius = test.number_input()
    print(test.get_circumference(radius))
    print(test.get_circle_area(radius))
    print(test.PI)

    print('메인의 __name__ 출력')
    print(__name__)
    print()
    
if __name__ == '__main__':
    main()