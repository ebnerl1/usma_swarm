# Mapping

As the drones received data, they sent it to the ground station which would then update the KML output. Each time it was updated, the ground station would overwrite and data already there. To write to the KML file, we used the Python library SimpleKML. This had all the functionality that we needed, including writing points, lines, and images.

## **Contour Lines**
Contour line mapping was fairly simple. We already had the data in a graph data structure, so to display the contour line, we would iterate over every vertex in the graph and write a point to the KML file. Then, we would iterate over each edge in the graph and write a line between its two points.

## **Heatmap**
The heatmap was a little more difficult. Since we weren't using the Google Earth JS API, we could not use heatmap functionality inherent to Google Earth. We tried a few different libraries, but found that they weren't working well. To solve the issue, we decided to create the heatmap ourselves. To do this, we would store an array of brightness values for each pixel, color the brightness values according to a gradient, write the pixels to an image, then add that image to the KML File.

After we got this working, we had to figure out how to modify the brightness values in a way that made sense. Since the drones collected data at a certain gps coordinate, this would only really cover 1 pixel for each data point. We decided to add a circular propagation to the points, so they affect a roughly 30 pixel radius, with decreasing weight as it got farther out. When the value propagated out, we didn't want to overwrite data already collected, so we would just average what was already there with the new data that was collected. 

We decided to go with this algorithm because it was fast. We were receiving many data points per second, and needed this to run fast. The proper way would be to mathematically calculate the heatmap for the entire image at each update with some sort of fractal. Since we needed this to take about 5 ms, we decided to limit each update to only modifying a few thousand brightness values in an array, which is very fast. 