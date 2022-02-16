import { FullscreenOutlined, FullscreenExitOutlined } from '@ant-design/icons';
import { Layout, Row, Col, Menu, Button, Modal } from 'antd';
import { Link } from 'umi';
import React, { Component } from 'react';

const { Header, Content, Footer } = Layout;

class Index extends Component {
  state = { fullscreen: false };
  modal = false;

  componentDidMount() {
    const that = this;
    let lastWidth = 0;

    function resizeHandle(width) {
      if (!that.modal && width < 576 && width !== lastWidth) {
        that.modal = true;
        lastWidth = width;
        Modal.warning({
          title: '显示窗口过窄',
          content: '部分内容可能无法完美显示，请调整窗口大小或者将手机转为横屏',
          onOk: () => {
            that.modal = false;
          },
        });
      }
      if (that.modal && width >= 576) {
        Modal.destroyAll();
      }
    }

    resizeHandle(window.innerWidth);

    window.addEventListener('resize', () => {
      resizeHandle(window.innerWidth);
    }, false);
  }

  fullScreen() {
    let element = document.documentElement;
    if (element.requestFullscreen) {
      element.requestFullscreen();
    } else if (element.msRequestFullscreen) {
      element.msRequestFullscreen();
    } else if (element.mozRequestFullScreen) {
      element.mozRequestFullScreen();
    } else if (element.webkitRequestFullscreen) {
      element.webkitRequestFullscreen();
    }
    this.setState({
      fullscreen: true,
    });
  }

  exitFullscreen() {
    if (document.exitFullscreen) {
      document.exitFullscreen();
    } else if (document.msExitFullscreen) {
      document.msExitFullscreen();
    } else if (document.mozCancelFullScreen) {
      document.mozCancelFullScreen();
    } else if (document.webkitExitFullscreen) {
      document.webkitExitFullscreen();
    }
    this.setState({
      fullscreen: false,
    });
  }

  render() {
    const fullButton = (
      <Button type='link' onClick={() => this.fullScreen()} size={'large'}>
        <FullscreenOutlined style={{ color: 'black' }} />
      </Button>
    );
    const unFullButton = (
      <Button type='link' onClick={() => this.exitFullscreen()} size={'large'}>
        <FullscreenExitOutlined style={{ color: 'black' }} />
      </Button>
    );
    return (
      <div>
        <Layout>
          <Header style={{ position: 'fixed', zIndex: 6, width: '100%', background: '#fff', paddingRight: 10 }}>
            <Row>
              <Col lg={4} md={5} sm={7} xs={14}>
                <div>
                  <h3>Radar Displayer</h3>
                </div>
              </Col>
              <Col lg={17} md={16} sm={13} xs={3}>
                <Menu theme='light' mode='horizontal' selectedKeys={[window.location.pathname]}>
                  <Menu.Item key='/'>
                    <Link to='/'>status</Link>
                  </Menu.Item>
                  <Menu.Item key='/calibrate'>
                    <Link to='/calibrate'>calibrate</Link>
                  </Menu.Item>
                </Menu>
              </Col>
              <Col md={3} sm={4} xs={7}>
                <div style={{ alignItems: 'right', float: 'right', marginRight: '5%' }}>
                  {
                    this.state.fullscreen ?
                      unFullButton :
                      fullButton
                  }
                </div>
              </Col>
            </Row>
          </Header>
          <Content className='site-layout' style={{ marginTop: 20 }}>
            <div className='site-layout-background'
                 style={{ padding: 24, paddingTop: 48, paddingBottom: 0, height: 'calc(100vh - 90px)', overflow: 'hidden' }}>
              {this.props.children}
            </div>
          </Content>
          <Footer style={{ textAlign: 'center' }}>Radar Displayer ©2022 Created by <a
            href={'https://github.com/bismarckkk'}>Bismarckkk</a></Footer>
        </Layout>
      </div>
    );
  }
}

export default Index;
