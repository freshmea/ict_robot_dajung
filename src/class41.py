def main():
    # power = lambda x: x * x
    # under_3 = lambda x: x < 3
    
    li = [1,2,3,4,5]
    # output_power = map(power, li)
    # output_under_3 = filter(under_3, li)
    
    output_power = map(lambda x: x*x, li)
    output_under_3 = filter(lambda x: x < 3, li)

    print(output_power)
    print(output_under_3)
    print(list(output_power))
    print(list(output_under_3))

if __name__ == '__main__':
    main()
