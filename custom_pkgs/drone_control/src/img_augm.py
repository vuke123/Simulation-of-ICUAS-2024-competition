from PIL import Image
import os
import numpy as np
import random

def generate_random_digits(num_digits):
    # Generate a random number with `num_digits` digits
    range_start = 10**(num_digits-1)
    range_end = (10**num_digits)-1
    return random.randint(range_start, range_end)


def augment_images(folder_path, num_augmented=10):
    images = [os.path.join(folder_path, f) for f in os.listdir(folder_path) if f.endswith('.jpg')]
    for img_path in images:
        img = Image.open(img_path)
	base_name = f[:7]

        # Generate rotated and skewed images
        for i in range(num_augmented):
            # Rotate image
            rand_num = generate_random_digits(6)            
            rotated = img.rotate(np.random.randint(-30, 30))
            rotated.save(os.path.join(folder_path, f"{base_name}{rand_num}.jpg"))


            # Skew image (using shear)
            skew_level = np.random.uniform(-0.2, 0.2)
            width, height = img.size
            xshift = abs(skew_level) * width
            new_width = width + int(round(xshift))
            rand_num = generate_random_digits(6)

            skew_img = img.transform((new_width, height), Image.AFFINE,
                                     (1, skew_level, -xshift if skew_level > 0 else 0, 0, 1, 0),
                                     Image.BICUBIC)
            skew_img.save(os.path.join(folder_path, f"{base_name}{rand_num}.jpg"))

# Example usage:
augment_images('./images')

