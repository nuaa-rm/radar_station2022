import React, { Component, createRef } from 'react';
import { connect } from 'umi';
import {
  ModalForm,
  ProFormSelect,
} from '@ant-design/pro-form';
import {Renderer as CanvasRenderer} from '@antv/g-canvas'
import {Canvas, Circle, Polygon, Text} from "@antv/g";
import {displayerBackend} from "../displayerBackend";

@connect(({ configProvider }) => ({
  configProvider,
}))
class CameraView extends Component {
  state = {camera: '', modalShow: false, aspectRatio: 4. / 3., scale: 1, offset: [0, 0]};
  container = createRef();
  renderer = new CanvasRenderer();
  canvas = null
  image = null
  shapes = {}
  lastHeight = 0
  lastWidth = 0

  onModalShowChange(e) {
    this.setState({modalShow: e})
  }

  componentDidMount() {
    const aspectRatio = this.state.aspectRatio;
    let width = this.props.width
    let height = this.props.height
    let targetWidth = height * aspectRatio;
    if (width < targetWidth) {
      targetWidth = width;
      height = targetWidth / aspectRatio;
    }
    this.lastHeight = height
    this.lastWidth = width
    this.canvas = new Canvas({
      container: 'cameraShape',
      width: this.props.width,
      height: this.props.height,
      renderer: this.renderer,
    });
    const that = this
    displayerBackend.onCameraShapeUpdate((shapes)=>{that.updateShapes(shapes)})
    displayerBackend.onViewSet((_1, _2, _3)=>{that.viewSet(_1, _2, _3)})
  }

  componentDidUpdate(prevProps, prevState, snapshot) {
    const aspectRatio = this.state.aspectRatio;
    let width = this.props.width
    let height = this.props.height
    let targetWidth = height * aspectRatio;
    if (width < targetWidth) {
      targetWidth = width;
      height = targetWidth / aspectRatio;
    }
    this.canvas.resize(targetWidth, height);
    const shapeKeys = Object.keys(this.shapes)
    for (let i = 0; i < shapeKeys.length; i++) {
      const shape = this.shapes[shapeKeys[i]]
      if (shape.main) {
        if (shape.shapeType === 'point') {
          shape.main.style.x *= targetWidth / this.lastWidth;
          shape.main.style.y *= height / this.lastHeight;
        } else if (shape.shapeType === 'polygon') {
          shape.main.style.points = shape.main.style.points.map(p => [
            p[0] * targetWidth / this.lastWidth,
            p[1] * height / this.lastHeight
          ])
        }
      }
      if (shape.text) {
        shape.text.style.x *= targetWidth / this.lastWidth;
        shape.text.style.y *= height / this.lastHeight;
      }
    }
    this.lastHeight = height
    this.lastWidth = targetWidth
  }

  updateShapes = (shapes) => {
    const aspectRatio = this.state.aspectRatio;
    let width = this.props.width
    let height = this.props.height
    let targetWidth = height * aspectRatio;
    if (width < targetWidth) {
      targetWidth = width;
      height = targetWidth / aspectRatio;
    }
    width = targetWidth;
    let reload = false;
    for (let i = 0; i < shapes.length; i++) {
      const shape = shapes[i];
      if (this.shapes[shape.id]) {
        if (shape.data.length === 0) {
          this.canvas.removeChild(this.shapes[shape.id].main);
          this.shapes[shape.id].main = null;
          if (this.shapes[shape.id].text) {
            this.canvas.removeChild(this.shapes[shape.id].text);
            this.shapes[shape.id].text = null;
          }
        } else {
          if (this.shapes[shape.id].shapeType === 'point') {
            if (this.shapes[shape.id].text) {
              if (shape.text) {
                this.shapes[shape.id].text.style.x = shape.data[0][0] * width - 1;
                this.shapes[shape.id].text.style.y = shape.data[0][1] * height;
                this.shapes[shape.id].text.style.text = shape.text;
              } else {
                this.canvas.removeChild(this.shapes[shape.id].text);
                this.shapes[shape.id].text = null;
              }
            } else {
              if (shape.text) {
                this.shapes[shape.id].text = new Text({
                  style: {
                    x: shape.data[0][0] * width - 1,
                    y: shape.data[0][1] * height,
                    text: shape.text,
                    fontSize: 12,
                    fontFamily: 'Microsoft YaHei',
                    fontWeight: 'bold',
                    textAlign: "center",
                    textBaseline: "middle",
                    fill: "#fff",
                    zIndex: 3
                  },
                });
                this.canvas.appendChild(this.shapes[shape.id].text);
              }
            }
            this.shapes[shape.id].main.style.x = shape.data[0][0] * width;
            this.shapes[shape.id].main.style.y = shape.data[0][1] * height;
            this.shapes[shape.id].main.style.fill = shape.color;
          } else if (this.shapes[shape.id].shapeType === 'polygon') {
            let x = width;
            let y = height;
            for (let j = 0; j < shape.data.length; j++) {
              const bx = shape.data[j][0] * width
              const by = shape.data[j][1] * height
              if (bx < x) {
                x = bx
              }
              if (by < y) {
                y = by
              }
            }
            if (this.shapes[shape.id].text) {
              if (shape.text) {
                this.shapes[shape.id].text.style.x = x;
                this.shapes[shape.id].text.style.y = y;
                this.shapes[shape.id].text.style.text = shape.text;
              } else {
                this.canvas.removeChild(this.shapes[shape.id].text);
                this.shapes[shape.id].text = null;
              }
            } else {
              if (shape.text) {
                this.shapes[shape.id].text = new Text({
                  style: {
                    x: x - 4,
                    y: y - 4,
                    text: shape.text,
                    fontSize: 16,
                    fontFamily: 'Microsoft YaHei',
                    fontWeight: 'bold',
                    textBaseline: "bottom",
                    fill: "#fff",
                    stroke: "#000",
                    zIndex: 3
                  },
                });
                this.canvas.appendChild(this.shapes[shape.id].text);
              }
            }
            this.shapes[shape.id].main.style.points = shape.data.map(item => [item[0] * width, item[1] * height]);
            this.shapes[shape.id].main.style.fill = shape.color;
            this.shapes[shape.id].main.style.stroke = shape.color;
          }
        }
      } else {
        this.shapes[shape.id] = {};
        if (shape.data.length === 1) {
          shape.shapeType = 'point'
        } else if (shape.data.length > 1) {
          shape.shapeType = 'polygon'
        }
        if (shape.shapeType === 'point') {
          if (shape.text) {
            this.shapes[shape.id].text = new Text({
              style: {
                x: shape.data[0][0] * width - 1,
                y: shape.data[0][1] * height,
                text: shape.text,
                fontSize: 12,
                fontFamily: 'Microsoft YaHei',
                fontWeight: 'bold',
                textAlign: "center",
                textBaseline: "middle",
                fill: "#fff",
                zIndex: 3
              },
            });
            this.canvas.appendChild(this.shapes[shape.id].text);
          }
          this.shapes[shape.id].main = new Circle({
            style: {
              x: shape.data[0][0] * width,
              y: shape.data[0][1] * height,
              r: 8,
              fill: shape.color,
              zIndex: 2
            },
          });
          this.canvas.appendChild(this.shapes[shape.id].main);
          this.shapes[shape.id].shapeType = 'point';
        } else if (shape.shapeType === 'polygon') {
          let x = width;
          let y = height;
          for (let j = 0; j < shape.data.length; j++) {
            const bx = shape.data[j][0] * width
            const by = shape.data[j][1] * height
            if (bx < x) {
              x = bx
            }
            if (by < y) {
              y = by
            }
          }
          if (shape.text) {
            this.shapes[shape.id].text = new Text({
              style: {
                x: x - 4,
                y: y - 4,
                text: shape.text,
                fontSize: 16,
                fontFamily: 'Microsoft YaHei',
                fontWeight: 'bold',
                textBaseline: "bottom",
                fill: "#fff",
                stroke: "#000",
                zIndex: 3
              },
            });
            this.canvas.appendChild(this.shapes[shape.id].text);
          }
          this.shapes[shape.id].main = new Polygon({
            style: {
              points: shape.data.map(item => [item[0] * width, item[1] * height]),
              fill: shape.color,
              fillOpacity: 0.3,
              stroke: shape.color,
              lineWidth: 2,
              zIndex: 2
            },
          });
          this.canvas.appendChild(this.shapes[shape.id].main);
          this.shapes[shape.id].shapeType = 'polygon';
        }
        reload = true
      }
      if (reload) {
        const that = this;
        setTimeout(()=>{
          that.updateShapes(shapes)
        }, 100)
      }
    }
  }

  onSubmit(e) {
    let aspectRatio = 4. / 3.;
    if (e.camera !== '') {
      aspectRatio = this.props.configProvider.calibrator.cameras[e.camera].aspectRatio;
    }
    displayerBackend.cameraSelect(e.camera)
    this.setState({
      camera: e.camera,
      modalShow: false,
      aspectRatio
    })
    // this.resize(aspectRatio)
    this.shapes = {}
    this.canvas.removeChildren()
  }

  viewSet(camera, scale, offset) {
    this.onSubmit({camera})
    if (scale < 1) {
      scale = 1;
    }
    this.setState({
      scale,
      offset: offset.map(e=>(e - 0.5)),
    })
  }

  render() {
    let uri;
    if (this.state.camera === '') {
      uri = require('../assets/noCamera.png');
    } else {
      uri = '/api/camera?cam=' + encodeURIComponent(this.state.camera)
    }
    let cameras = {}
    const cameraList = Object.keys(this.props.configProvider.calibrator.cameras)
    for (let i = 0; i < cameraList.length; i++) {
      cameras[cameraList[i]] = cameraList[i]
    }

    const aspectRatio = this.state.aspectRatio;
    let width = this.props.width
    let height = this.props.height
    let targetWidth = height * aspectRatio;
    if (width < targetWidth) {
      targetWidth = width;
      height = targetWidth / aspectRatio;
    }
    return (
      <div style={{height: '100%', width: '100%', overflow: 'hidden'}} ref={this.container}>
        <div style={{overflow: 'hidden', height: height, width: targetWidth, background: '#fff '}}>
          <div style={{transform: `scale(${this.state.scale}, ${this.state.scale}) translate(${this.state.offset[0]}%, ${this.state.offset[1]}%)`, height: '100%', width: '100%'}}>
            <div style={{ position: 'relative', height: height, width: targetWidth, cursor: 'pointer' }}
                 onClick={()=>{this.onModalShowChange(true)}}>
              <div style={{ position: 'absolute', top: '0px', left: '0px', width: '100%', height: '100%', zIndex: 1 }}>
                <div id="cameraShape" />
              </div>
              <div style={{ position: 'absolute', top: '0px', left: '0px', width: '100%', height: '100%', zIndex: 0 }}>
                <img src={uri} alt="camera" style={{height: height, width: targetWidth}} />
              </div>
            </div>
          </div>
        </div>
        <ModalForm
          title="Select Camera"
          onFinish={async values=>{this.onSubmit(values)}}
          visible={this.state.modalShow}
          onVisibleChange={e=>this.onModalShowChange(e)}
        >
          <ProFormSelect
            name="camera"
            label="Camera"
            placeholder="Please Select Camera"
            valueEnum={cameras}
          />
        </ModalForm>
      </div>
    );
  }
}

export default CameraView;
