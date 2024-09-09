import cv2 # type: ignore
import numpy as np
from scipy.spatial import distance # type: ignore

# Load the image
p = '/home/user/Documents/saved/take3.jpg'
image = cv2.imread(p)

# Create a circular mask
height, width = image.shape[:2]
center = (width // 2, height // 2)
radius = 330

threshold = 100
min_contour_area = .1  # Minimum area of spots to be considered (adjust as needed)
cluster_distance_threshold = 50  # Maximum distance to consider spots part of the same cluster (adjust as needed)
min_cluster_size = 4  # Minimum number of spots in a cluster to be considered

mask = np.zeros((height, width), dtype=np.uint8)
cv2.circle(mask, center, radius, 255, -1)

# Apply the mask to get a circular cropped image
masked_image = cv2.bitwise_and(image, image, mask=mask)

# Convert to grayscale
gray = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)

# Apply a binary threshold to get a binary image
_, binary = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY_INV)  # Adjust threshold as needed

# Find contours
contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Filter contours based on size and draw green dots at each valid spot
centroids = []
for contour in contours:
    if cv2.contourArea(contour) > min_contour_area:
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            centroids.append((cX, cY))
            # Draw green dot at each valid spot
            cv2.circle(masked_image, (cX, cY), 3, (0, 255, 0), -1)

# Function to calculate the distance from a point to the center
def distance_from_center(point):
    return distance.euclidean(point, center)

# Filter centroids to find clusters of spots close together
clusters = []
for i, centroid in enumerate(centroids):
    cluster = [centroid]
    for j, other_centroid in enumerate(centroids):
        if i != j and distance.euclidean(centroid, other_centroid) < cluster_distance_threshold:
            cluster.append(other_centroid)
    if len(cluster) >= min_cluster_size:
        clusters.append(cluster)

# Draw red dots for centroids of all valid clusters
for cluster in clusters:
    for centroid in cluster:
        cv2.circle(masked_image, centroid, 5, (0, 0, 255), -1)

# Find the cluster closest to the center
if clusters:
    closest_cluster = min(clusters, key=lambda c: distance_from_center(np.mean(c, axis=0)))
    avg_cX, avg_cY = np.mean(closest_cluster, axis=0).astype(int)
    print(f"Centroid of the closest cluster: ({avg_cX}, {avg_cY})")
    
    # Draw a purple dot on the centroid of the closest cluster
    cv2.circle(masked_image, (avg_cX, avg_cY), 7, (255, 0, 255), -1)
else:
    print("No clusters found.")

# Display the circular masked image with the detected spots and centroids
cv2.imshow('Circular Masked Image with Centroids', masked_image)
cv2.waitKey(0)
cv2.destroyAllWindows()

