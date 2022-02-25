import React, {Component} from 'react';
import {Canvas, Circle, Image, Polygon, Text} from '@antv/g';
import {Renderer as WebGLRenderer} from '@antv/g-webgl';

class Minimap extends Component {
  renderer = new WebGLRenderer();
  canvas = null
  image = null
  shapes = {}

  componentDidMount() {
    this.canvas = new Canvas({
      container: 'minimap',
      width: this.props.width,
      height: this.props.height,
      renderer: this.renderer,
    });
    this.image = new Image({
      style: {
        width: this.props.width,
        height: this.props.height,
        x: 0,
        y: 0,
        zIndex: 0,
        img: require('../assets/minimap.png')
      },
    })
    this.canvas.appendChild(this.image);
    const that = this
    setTimeout(()=>{
      that.updateShapes({
        test1: {color: 'red', id: 'test1', text: 'test', data: [[0.2, 0.5], [0.6, 0.7], [0.2, 0.7]], shapeType: 'polygon'},
        test2: {color: 'green', data: [[0.3, 0.5]], text: '1', shapeType: 'point', id: 'test2'}
      })

    }, 5000)
  }

  componentDidUpdate(prevProps, prevState, snapshot) {
    this.canvas.resize(this.props.width, this.props.height);
    this.image.style.height = this.props.height;
    this.image.style.width = this.props.width;
    const shapeKeys = Object.keys(this.shapes)
    for (let i = 0; i < shapeKeys.length; i++) {
      const shape = this.shapes[shapeKeys[i]]
      if (shape.main) {
        if (shape.shapeType === 'point') {
          shape.main.style.x *= this.props.width / prevProps.width;
          shape.main.style.y *= this.props.height / prevProps.height;
        } else if (shape.shapeType === 'polygon') {
          shape.main.style.points = shape.main.style.points.map(p => [
            p[0] * this.props.width / prevProps.width,
            p[1] * this.props.height / prevProps.height
          ])
        }
      }
      if (shape.text) {
        shape.text.style.x *= this.props.width / prevProps.width;
        shape.text.style.y *= this.props.height / prevProps.height;
      }
    }
  }

  updateShapes = (shapes) => {
    const width = this.props.width;
    const height = this.props.height;
    for (let i = 0; i < Object.keys(shapes).length; i++) {
      const shape = shapes[Object.keys(shapes)[i]];
      if (this.shapes[shape.id]) {
        if (shape.data.length === 0) {
          this.canvas.removeChild(this.shapes[shape.id].main);
          this.shapes[shape.id].main = null;
          if (this.shapes[shape.id].text) {
            this.canvas.removeChild(this.shapes[shape.id].text);
            this.shapes[shape.id].text = null;
          }
        } else {
          if (shape.shapeType === 'point') {
            if (this.shapes[shape.id].text) {
              if (shape.text) {
                this.shapes[shape.id].text.style.x = shape.data[0].x;
                this.shapes[shape.id].text.style.y = shape.data[0].y;
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
      }
    }
  }

  render() {
    return (
      <div id="minimap" />
    );
  }
}

export default Minimap;
