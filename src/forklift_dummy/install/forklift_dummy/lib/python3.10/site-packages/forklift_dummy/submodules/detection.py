from .centroidtracker import CentroidTracker
import torch
import cv2

class Detect():

    def __init__(self):

        self.ct = CentroidTracker()
        self.rects = []
        self.boxes_id = []
        self.centroid_box = []
        self.active_id = 0
        self.locked_box = None
        self.n = 0

        self.model = torch.hub.load(
            "/home/ros2/ros2_ws/src/forklift_dummy/forklift_dummy/submodules/yolov5",
            "custom",
            path="/home/ros2/ros2_ws/src/forklift_dummy/forklift_dummy/submodules/yolov5/best.pt",
            source="local",
        )  # local repo

        self.model.conf = 0.6  # confidence threshold (0-1)
        self.model.iou = 0.7  # NMS IoU threshold (0-1)

    def plot_boxes(self, results, frame_p):
        # print(results)

        labels, cord = (
            results.xyxyn[0][:, -1].cpu().numpy(),
            results.xyxyn[0][:, :-1].cpu().numpy(),
        )
        n = len(labels)
        x_shape, y_shape = frame_p.shape[1], frame_p.shape[0]
        self.rects = []
        # print(cord)
        for i in range(len(labels)):
            row = cord[i]
            if row[4] >= 0.2:
                x1, y1, x2, y2 = (
                    int(row[0] * x_shape),
                    int(row[1] * y_shape),
                    int(row[2] * x_shape),
                    int(row[3] * y_shape),
                )
                self.rects.append([x1, y1, x2, y2])
 
        objects = self.ct.update(self.rects)

        # print(type(objects))
        # print(objects)

        self.boxes_id = []
        self.centroid_box = []

        for objectID, centroid in objects.items():
            # draw both the ID of the object and the centroid of the
            # object on the output frame
            text = "ID {}".format(objectID)
            self.boxes_id.append(objectID)
            self.centroid_box.append(centroid)
            cv2.putText(
                frame_p,
                text,
                (centroid[0] - 10, centroid[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )
            cv2.circle(frame_p, (centroid[0], centroid[1]), 4, (0, 255, 0), -1)

        if len(self.boxes_id) > 0:
            # print(self.boxes_id)

            for i in self.boxes_id:
                bgr = (0, 0, 255)

                # print(self.boxes_id.index(i))
                # print(self.rects)
                # print(self.active_id)
                x = self.centroid_box[self.boxes_id.index(i)][0]
                y = self.centroid_box[self.boxes_id.index(i)][1]

                # print(self.locked_box)
                if len(self.rects) > 0:
                    for startX, startY, endX, endY in self.rects:
                        # use the bounding box coordinates to derive the centroid
                        cX = int((startX + endX) / 2.0)
                        cY = int((startY + endY) / 2.0)

                        if x == cX and y == cY:
                            x1 = startX

                            y1 = startY

                            x2 = endX

                            y2 = endY

                    if i == self.active_id:
                        bgr = (0, 255, 0)

                    if self.locked_box == i:
                        overlay = frame_p.copy()
                        cv2.rectangle(
                            overlay,
                            (x1, y1),
                            (x2, y2),
                            (
                                255,
                                0,
                                0,
                            ),
                            -1,
                        )
                        alpha = 0.4
                        frame_p = cv2.addWeighted(overlay, alpha, frame_p, 1 - alpha, 0)
                        self.distance = (479.5 * 100) / (x2 - x1) - 7.835
                        # print((633.15 * (x2-x1))/100)
                        # print(self.distance)
                        self.mid_point = (
                            f"{x1 + (x2 - x1)/2},{y2 - (y2 - y1)/2},{x1},{x2}"
                        )
                        # print(self.mid_point)
                        # print((x1, y1), (x2, y2))

                    else:
                        cv2.rectangle(frame_p, (x1, y1), (x2, y2), bgr, 2)

                # cv.putText(frame, f"Object{i}", (x1, y1), cv.FONT_HERSHEY_SIMPLEX, 0.7, bgr, 1)

        self.n = len(self.boxes_id)

        return frame_p 

    def detection(self, img, active, locked):

        self.active_id = active

        self.locked_box = locked

        frame_brg = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        frame = cv2.resize(frame_brg, (1280, 720))

        results = self.model(frame)

        frame_final = self.plot_boxes(results, frame)

        return frame_final, self.boxes_id, self.centroid_box
















