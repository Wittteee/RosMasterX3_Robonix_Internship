import io
from unittest.mock import Mock, patch

import numpy as np
import pytest

import download_yolox
import run_object_detection
import run_visual_nav
from x3_nav.vision.yolox import ObjectBox, ObjectObserver, YOLOXDetector, select_unique


def raw_output():
    raw = np.zeros((1, 8400, 85), np.float32)
    # First cell stride 8: center=(160,80), size=(80,40).
    raw[0, 0, :4] = [20, 10, np.log(10), np.log(5)]
    raw[0, 0, 4:6] = [.9, .8]
    return raw


def test_decode_geometry_score_and_normalized_detection():
    boxes = YOLOXDetector.decode(raw_output(), 320, 160, 2)
    assert len(boxes) == 1
    box = boxes[0]
    assert box.xyxy == pytest.approx((60, 30, 100, 50))
    assert box.confidence == pytest.approx(.72)
    assert box.label == 'person'
    det = box.as_detection()
    assert det.offset_x == pytest.approx(-.5)
    assert det.size == pytest.approx(800 / 51200)
    assert det.source == 'yolox-opencv'


def test_class_aware_nms():
    raw = raw_output()
    raw[0, 1] = raw[0, 0]
    raw[0, 1, 0] -= 1  # same box from next grid cell
    assert len(YOLOXDetector.decode(raw, 640, 640, 1)) == 1
    raw[0, 1, 5] = 0
    raw[0, 1, 6] = .8
    assert len(YOLOXDetector.decode(raw, 640, 640, 1)) == 2


def test_no_boxes_and_invalid_model_output():
    assert YOLOXDetector.decode(np.zeros((1,8400,85)),640,480,1) == []
    with pytest.raises(ValueError):
        YOLOXDetector.decode(np.zeros((1,8400,84)),640,480,1)
    raw = raw_output()
    raw[0,0,0] = np.nan
    with pytest.raises(ValueError):
        YOLOXDetector.decode(raw,640,480,1)


def test_preprocessing_rgb_top_left_padding(tmp_path):
    model = tmp_path / 'model.onnx'
    model.touch()
    net = Mock()
    net.forward.return_value = np.zeros((1,8400,85),np.float32)
    with patch('cv2.dnn.readNetFromONNX', return_value=net):
        detector = YOLOXDetector(model)
    frame = np.full((320,640,3),[10,20,30],np.uint8)
    assert detector.detect(frame) == []
    blob = net.setInput.call_args.args[0]
    assert blob.shape == (1,3,640,640)
    assert blob[0,:,0,0].tolist() == [30,20,10]
    assert blob[0,:,400,0].tolist() == [114,114,114]


@pytest.mark.parametrize('count',[0,2])
def test_ambiguity_refuses_target(count):
    box = ObjectBox('bottle',.9,(10,10,30,50),640,480)
    with pytest.raises(ValueError):
        select_unique([box]*count,'bottle')


def test_observer_reacquires_and_checks_resolution():
    detector = Mock()
    detector.detect.return_value = [ObjectBox('bottle',.9,(100,100,140,200),640,480)]
    observer = ObjectObserver(detector,'0','bottle',(500,320,0),(480,640))
    with patch('x3_nav.vision.yolox.read_frame',return_value=np.zeros((480,640,3))) as read:
        assert observer() > 0
        observer()
        assert read.call_count == 2
    with patch('x3_nav.vision.yolox.read_frame',return_value=np.zeros((240,320,3))):
        with pytest.raises(ValueError,match='dimensions'):
            observer()


def test_observer_rejects_stale_inference_and_recorded_video():
    with pytest.raises(ValueError,match='local camera'):
        ObjectObserver(Mock(),'recorded.mp4','bottle',(500,320,0),(480,640))
    observer = ObjectObserver(Mock(),'0','bottle',(500,320,0),(480,640))
    with patch('x3_nav.vision.yolox.read_frame',return_value=np.zeros((480,640,3))), \
         patch('x3_nav.vision.yolox.time.monotonic',side_effect=[0,3]):
        with pytest.raises(ValueError,match='2-second'):
            observer()


def test_missing_model_fails_without_network(tmp_path,capsys):
    with patch('roslibpy.Ros', side_effect=AssertionError('network')):
        assert run_object_detection.main(['--image','missing.jpg','--model',str(tmp_path/'missing.onnx'),
                                          '--output-dir',str(tmp_path)]) == 2
    assert 'Model missing' in capsys.readouterr().out


def test_yolox_demo_rejected_before_provider():
    with pytest.raises(SystemExit):
        run_visual_nav.main(['--demo','--detector','yolox','--label','bottle'])


def test_checksum_failure_preserves_existing_model(tmp_path):
    destination = tmp_path/'model.onnx'
    destination.write_bytes(b'existing')
    with patch('urllib.request.urlopen',return_value=io.BytesIO(b'corrupt')):
        with pytest.raises(ValueError,match='mismatch'):
            download_yolox.download(destination)
    assert destination.read_bytes() == b'existing'
    assert not destination.with_suffix('.onnx.part').exists()


def test_image_cli_writes_boxes_without_robot(tmp_path, capsys):
    import json
    detector = Mock()
    detector.detect.return_value = [ObjectBox('bottle',.9,(100,100,140,200),640,480)]
    with patch('run_object_detection.YOLOXDetector',return_value=detector), \
         patch('run_object_detection.read_frame',return_value=np.zeros((480,640,3),np.uint8)), \
         patch('roslibpy.Ros',side_effect=AssertionError('network')):
        assert run_object_detection.main(['--image','test.jpg','--label','bottle',
                                          '--output-dir',str(tmp_path)]) == 0
    data = json.loads(capsys.readouterr().out)
    assert data['candidate_detection']['source'] == 'yolox-opencv'
    assert not data['navigation_goal_sent']
    assert list(tmp_path.glob('*.jpg')) and list(tmp_path.glob('*.json'))


@pytest.mark.parametrize('target_present',[True,False])
def test_yolox_provider_preview_never_executes(tmp_path, target_present):
    from x3_nav.bridge.simulation_provider import SimulationProvider
    detector = Mock()
    detector.detect.return_value = ([ObjectBox('bottle',.9,(300,100,340,200),640,480)]
                                    if target_present else [])
    api = Mock(wraps=SimulationProvider())
    with patch('x3_nav.vision.yolox.YOLOXDetector',return_value=detector), \
         patch('x3_nav.vision.yolox.read_frame',return_value=np.zeros((480,640,3),np.uint8)), \
         patch('x3_nav.bridge.robonix_navigation.RobonixNavigation',return_value=api):
        code = run_visual_nav.main(['--detector','yolox','--camera','0','--label','bottle',
                                    '--provider-dir','test-provider','--fx','500','--cx','320',
                                    '--camera-yaw-deg','0','--image-width','640','--image-height','480',
                                    '--output-dir',str(tmp_path)])
    assert code == (0 if target_present else 2)
    api.execute.assert_not_called()
    if target_present:
        assert detector.detect.call_count == 2
        api.prepare.assert_called_once()
    else:
        api.prepare.assert_not_called()
