import sys

def main():
    for value in sys.argv:
        print(value)
    # if sys.argv[1]  == 'default':
    #     print('this is default mode')
    # if sys.argv[1] == 'active':
    #     print('this is active mode')
    sys.exit()
    print(sys.version, sys.version_info, sys.copyright, sys.api_version, sys.getprofile())

if __name__ == '__main__':
    main()
