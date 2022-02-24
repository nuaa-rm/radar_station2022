import React, { Component, createRef } from 'react';
import { connect } from 'umi';
import {
  ModalForm,
  ProFormSelect,
} from '@ant-design/pro-form';

@connect(({ configProvider }) => ({
  configProvider,
}))
class CameraView extends Component {
  state = {camera: '', modalShow: false, aspectRatio: 4. / 3., width: 800, height: 600};
  container = createRef();

  onModalShowChange(e) {
    console.log(e)
    this.setState({modalShow: e})
  }

  onSubmit(e) {
    let aspectRatio = 4. / 3.;
    if (e.camera !== '') {
      aspectRatio = this.props.configProvider.calibrator.cameras[e.camera].aspectRatio;
    }
    this.setState({
      camera: e.camera,
      modalShow: false,
      aspectRatio
    })
    this.resize(aspectRatio)
  }

  resize(aspectRatio=null) {
    setTimeout(() => {
      let height = this.container?.current?.clientHeight;
      let width = this.container?.current?.clientWidth;
      console.log(width, height)
      if (height && width) {
        if (!aspectRatio) {
          aspectRatio = this.state.aspectRatio
        }
        let targetWidth = height * aspectRatio;
        if (width < targetWidth) {
          targetWidth = width;
          height = targetWidth / aspectRatio;
        }
        this.setState({
          width: targetWidth,
          height: height
        })
      }
    }, 50)
  }

  componentDidMount() {
    window.addEventListener('resize', ()=>{this.resize()})
    this.resize()
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
    return (
      <div style={{height: '100%', width: '100%'}} ref={this.container}>
        <img
          src={uri} alt="camera" style={{height: this.state.height, width: this.state.width, cursor: 'pointer'}}
          onClick={()=>{this.onModalShowChange(true)}}
        />
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
