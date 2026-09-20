from http import HTTPStatus
from typing import Any

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.error_envelope import ErrorEnvelope
from ...types import UNSET, Response


def _get_kwargs(
    *,
    scene: str,
) -> dict[str, Any]:

    params: dict[str, Any] = {}

    params["scene"] = scene

    params = {k: v for k, v in params.items() if v is not UNSET and v is not None}

    _kwargs: dict[str, Any] = {
        "method": "get",
        "url": "/v1/chassis/map_image",
        "params": params,
    }

    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ErrorEnvelope | None:
    if response.status_code == 400:
        response_400 = ErrorEnvelope.from_dict(response.json())

        return response_400

    if response.status_code == 404:
        response_404 = ErrorEnvelope.from_dict(response.json())

        return response_404

    if response.status_code == 409:
        response_409 = ErrorEnvelope.from_dict(response.json())

        return response_409

    if response.status_code == 502:
        response_502 = ErrorEnvelope.from_dict(response.json())

        return response_502

    if response.status_code == 504:
        response_504 = ErrorEnvelope.from_dict(response.json())

        return response_504

    if client.raise_on_unexpected_status:
        raise errors.UnexpectedStatus(response.status_code, response.content)
    else:
        return None


def _build_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> Response[ErrorEnvelope]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    *,
    client: AuthenticatedClient | Client,
    scene: str,
) -> Response[ErrorEnvelope]:
    """Fetch one scene's map as a PNG

     The only operation that does not answer with the JSON envelope on success: the body is the raw PNG.
    Failures still answer with the error envelope, so branch on the response content type, not on the
    status alone. Not available over WebSocket.

    Args:
        scene (str):

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        scene=scene,
    )

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    *,
    client: AuthenticatedClient | Client,
    scene: str,
) -> ErrorEnvelope | None:
    """Fetch one scene's map as a PNG

     The only operation that does not answer with the JSON envelope on success: the body is the raw PNG.
    Failures still answer with the error envelope, so branch on the response content type, not on the
    status alone. Not available over WebSocket.

    Args:
        scene (str):

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope
    """

    return sync_detailed(
        client=client,
        scene=scene,
    ).parsed


async def asyncio_detailed(
    *,
    client: AuthenticatedClient | Client,
    scene: str,
) -> Response[ErrorEnvelope]:
    """Fetch one scene's map as a PNG

     The only operation that does not answer with the JSON envelope on success: the body is the raw PNG.
    Failures still answer with the error envelope, so branch on the response content type, not on the
    status alone. Not available over WebSocket.

    Args:
        scene (str):

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        scene=scene,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    *,
    client: AuthenticatedClient | Client,
    scene: str,
) -> ErrorEnvelope | None:
    """Fetch one scene's map as a PNG

     The only operation that does not answer with the JSON envelope on success: the body is the raw PNG.
    Failures still answer with the error envelope, so branch on the response content type, not on the
    status alone. Not available over WebSocket.

    Args:
        scene (str):

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            client=client,
            scene=scene,
        )
    ).parsed
