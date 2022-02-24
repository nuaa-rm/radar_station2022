import React, { Component, createRef } from 'react';
import { connect } from 'dva';
import CameraView from '../components/cameraView';
import Hp from "../components/hp";

@connect(({ configProvider }) => ({
  configProvider,
}))
class Index extends Component {
  state = {
    height: 600,
    width: 800
  }
  container = createRef()

  resize() {
    const clientHeight = this.container?.current?.clientHeight;
    const clientWidth = this.container?.current?.clientWidth;
    if (clientHeight) {
      this.setState({
        height: clientHeight - 20,
        width: clientWidth - 20
      })
    }
  }

  componentDidMount() {
    this.resize();
    window.addEventListener('resize', ()=>{this.resize()});
  }

  render() {

    return (
      <div ref={this.container} style={{ height: '100%', width: '100%' }}>
        <img
          src={require('../assets/minimap.png')}
          style={{ height: this.state.height, width: this.state.height / 1.8, marginTop: 10, float: 'right', display: 'inline' }}
          alt="minimap"
        />
        <div style={{ height: this.state.height, width: this.state.height * 0.35, marginTop: 10, float: 'right', display: 'inline', marginRight: 10 }}>
          <Hp />
        </div>
        <div style={{
          height: this.state.height.toString() + 'px',
          width: (this.state.width - this.state.height / 1.1).toString() + 'px',
          paddingTop: 10,
        }}>
          <CameraView />
        </div>

      </div>
    );
  }
}

export default Index;
