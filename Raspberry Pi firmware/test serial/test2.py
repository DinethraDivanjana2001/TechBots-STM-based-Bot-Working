def generate_pascals_triangle(num_rows):
    triangle = []

    for row_num in range(num_rows):
        row = [1] * (row_num + 1)
        for j in range(1, row_num):
            row[j] = triangle[row_num - 1][j - 1] + triangle[row_num - 1][j]
        triangle.append(row)

    return triangle

def row_sum_pascals_triangle(row_number):
    triangle = generate_pascals_triangle(row_number + 1)  # Generate up to row_number
    row_sum = sum(triangle[row_number])
    return row_sum

# Example usage:
row_number = 3
triangle = generate_pascals_triangle(row_number + 1)
print("Pascal's Triangle:")
for row in triangle:
    print(row)

row_sum = row_sum_pascals_triangle(row_number)
print(f"\nSum of row {row_number}:", row_sum)