def power(item):
    return item * item

def under_3(item):
    return item < 3

def main():
    li = [1,2,3,4,5]
    
    # map 활용.
    output_map = map(power, li)
    print('map(power, li) 의 결과', output_map)
    print('map(power, li) 의 결과', list(output_map))
    
    
    # filter 활용.
    output_filter = filter(under_3, li)
    print('filter(power, li) 의 결과', output_filter)
    print('filter(power, li) 의 결과', list(output_filter))    

if __name__ == '__main__':
    main()
