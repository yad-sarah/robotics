import imageio
import numpy as np
def create_mp4_from_images(image_list, output_filename, fps):
    """
    Creates an MP4 video from a list of 3-dimensional NumPy arrays.

    Parameters:
    - image_list: A list of 3-dimensional NumPy arrays representing images.
    - output_filename: The filename for the output MP4 video.
    - fps: Frames per second for the output video.
    """
    with imageio.get_writer(output_filename, fps=fps) as writer:
        for img in image_list:
            # Ensure the image is in uint8
            if img.dtype != np.uint8:
                img = (img * 255).astype(np.uint8)
            writer.append_data(img)

    print(f'Video saved as {output_filename}')