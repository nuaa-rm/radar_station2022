import React, {Component} from 'react';
import { Canvas, Circle, Polygon, Image, Text } from '@antv/react-g';
import { Renderer as WebGLRenderer } from '@antv/g-webgl';
import { connect } from 'umi';

@connect(({ minimap }) => ({
  minimap,
}))
class Minimap extends Component {
  renderer = new WebGLRenderer();

  render() {
    const height = this.props.height
    const width = this.props.width
    const draw = [(
      <Image img={require('../assets/minimap.png')} x={0} y={0} height={height} width={width} zIndex={0} key="image" />
    )]
    const info = this.props.minimap
    const infoKeys = Object.keys(info)
    for (let i = 0; i < infoKeys.length; i++) {
      const it = info[infoKeys[i]]
      if (it.data.length > 1) {
        draw.push(<Polygon
          key={infoKeys[i]}
          fillOpacity={0.3}
          fill={it.color}
          stroke={it.color}
          lineWidth={2}
          points={it.data.map(p => [p[0] * width, p[1] * height])}
          zIndex={1}
        />)
        if (it.text) {
          let x = width;
          let y = height;
          for (let j = 0; j < it.data.length; j++) {
            const bx = it.data[j][0] * width
            const by = it.data[j][1] * height
            if (bx < x) {
              x = bx
            }
            if (by < y) {
              y = by
            }
          }
          draw.push(
            <Text
              key={infoKeys[i] + '-text'}
              x={x - 4}
              y={y - 4}
              text={it.text}
              fontSize={16}
              fontFamily={'Microsoft YaHei'}
              fontWeight={'bold'}
              textBaseline="bottom"
              stroke="#000"
              fill="#fff"
              zIndex={3}
            />
          )
        }
      } else {
        draw.push(
          <Circle
            key={infoKeys[i]}
            fill={it.color}
            stroke={it.color}
            r={8}
            x={it.data[0][0] * width}
            y={it.data[0][1] * height}
            zIndex={2}
          />
        )
        if (it.text) {
          draw.push(
            <Text
              key={infoKeys[i] + '-text'}
              x={it.data[0][0] * width - 1}
              y={it.data[0][1] * height}
              text={it.text}
              fontSize={12}
              fontFamily={'Microsoft YaHei'}
              fontWeight={'bold'}
              textAlign="center"
              textBaseline="middle"
              fill="#fff"
              zIndex={3}
            />
          )
        }
      }
    }
    return (
      <Canvas height={this.props.height} width={this.props.height / 1.8} renderer={this.renderer}>
        {draw}
      </Canvas>
    );
  }
}

export default Minimap;
